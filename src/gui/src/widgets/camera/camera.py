from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import pyqtSignal
from PyQt5.QtGui import QImage
from ..enums import CameraParams
from .hud import CameraOverlay

import cv2
import numpy as np


class CameraWidget(QtWidgets.QWidget):
    update_camera_signal = pyqtSignal(QtGui.QPixmap)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.has_frame = False
        self.show_depth = False
        self.numObj = 0
        self.detected_objects = np.zeros(7)

        # Lane
        self.center = None
        self.crosswalk = False
        self.stopline = False
        self.stopline_dist = None

        self.class_names = ["oneway", "highwayentrance", "stopsign", "roundabout", "park", "crosswalk", "noentry", "highwayexit", "priority", "lights", "block", "pedestrian", "car", "green light", "yellow light", "red light"]
        self.confidence_thresholds = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.65, 0.65, 0.65, 0.65, 0.7, 0.75, 0.65, 0.65, 0.65]

        self.COLOR_LIST = [
            (1, 1, 1), (0.098, 0.325, 0.850), (0.125, 0.694, 0.929), (0.556, 0.184, 0.494), (0.188, 0.674, 0.466),
            (0.933, 0.745, 0.301), (0.184, 0.078, 0.635), (0.300, 0.300, 0.300), (0.600, 0.600, 0.600), (0.000, 0.000, 1.000),
            (0.000, 0.500, 1.000), (0.000, 0.749, 0.749), (0.000, 1.000, 0.000)
        ]

        self.setup_ui()
        self.update_camera_signal.connect(self.update_camera_display)

    def setup_ui(self):
        self.camera_label = QtWidgets.QLabel(self)
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.camera_label.setStyleSheet("""
            background-color: rgba(255, 255, 255, 0.08);
        """)
        self.camera_label.setMinimumWidth(CameraParams.MIN_WIDTH.value)
        self.camera_label.setMinimumHeight(CameraParams.MIN_HEIGHT.value)

        self.hud = CameraOverlay(self.camera_label, self)
        self.hud.setMinimumWidth(CameraParams.MIN_WIDTH.value)
        self.hud.setMinimumHeight(CameraParams.MIN_HEIGHT.value)

        layout = QtWidgets.QVBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.camera_label, alignment=QtCore.Qt.AlignCenter)
        self.setLayout(layout)

    def update_camera_display(self, pixmap):
        self.camera_label.setPixmap(pixmap)
        self.has_frame = True

    def update_hud(self):
        self.hud.update_overlay()

    def process_camera_frame(self, cv_image):
        """Process RGB camera frame"""
        try:
            if self.show_depth:
                return

            cv_image = self.add_sign_detection_to_image(cv_image)
            cv_image = self.add_lane_detection_to_image(cv_image)
            cv_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)

            # Convert to QImage
            h, w, ch = cv_image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(cv_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            self.update_camera_signal.emit(pixmap)
        except Exception as e:
            print(f"Camera processing error: {e}")

    def process_depth_frame(self, depth_image):
        """Process depth camera frame"""
        try:
            if not self.show_depth:
                return
            # depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
            # depth_image = self.bridge.imgmsg_to_cv2(depth_frame, "mono16")  # real
            # depth_image = cv2.resize(depth_image, (self.camera_w, self.camera_h))
            # Apply normalization with a focus on closer objects
            depth_normalized = cv2.normalize(depth_image, None, 50, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_TURBO)  # TURBO colormap for better contrast
            depth_colored = self.add_sign_detection_to_image(depth_colored)
            depth_colored = self.add_lane_detection_to_image(depth_colored)

            # Convert to QImage
            h, w, ch = depth_colored.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(depth_colored.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            self.update_camera_signal.emit(pixmap)
        except Exception as e:
            print(f"Depth processing error: {e}")

    def add_sign_detection_to_image(self, image):
        for i in range(self.numObj):
            try:
                id = int(self.detected_objects[7 * i + 6])
            except Exception as e:
                print("Error in sign detection")
                print(e)
                return
            if self.detected_objects[7 * i + 5] < self.confidence_thresholds[id]:
                continue

            color_index = id % len(self.COLOR_LIST)
            color = tuple(int(c * 255) for c in self.COLOR_LIST[color_index])  # Scale color to [0, 255]

            mean_color = np.mean(color)
            text_color = (0, 0, 0) if mean_color > 127 else (255, 255, 255)

            confidence = self.detected_objects[7 * i + 5] * 100
            distance = self.detected_objects[7 * i + 4]
            text = f"{self.class_names[id]} {confidence:.1f}% {distance:.2f}m"

            label_size, baseLine = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

            x1 = int(self.detected_objects[7 * i])
            y1 = int(self.detected_objects[7 * i + 1])
            x2 = int(self.detected_objects[7 * i + 2])
            y2 = int(self.detected_objects[7 * i + 3])

            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2, lineType=cv2.LINE_AA)

            y = y1 - label_size[1] - baseLine
            if y < 0:
                y = 0
            x = x1
            if x + label_size[0] > image.shape[1]:
                x = image.shape[1] - label_size[0]

            txt_bk_color = tuple(int(c * 0.7) for c in color)
            cv2.rectangle(image, (x, y), (x + label_size[0], y + label_size[1] + baseLine), txt_bk_color, -1)

            cv2.putText(image, text, (x, y + label_size[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)

        return image

    def add_lane_detection_to_image(self, image):
        if self.center is None:
            return image
        # Draw the center line
        if image is None:
            return image
        cv2.line(image, (int(self.center), image.shape[0]), (int(self.center), int(0.8 * image.shape[0])), (0, 0, 255), 5)
        cv2.putText(image, f"center: {self.center:.2f}",
                    (int(image.shape[1] * 0.05), int(image.shape[0] * 0.1)),
                    cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        # Add text if stopline or crosswalk is detected
        if self.stopline:
            cv2.putText(image, "Stopline detected!",
                        (int(image.shape[1] * 0.05), int(image.shape[0] * 0.3)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        if self.crosswalk:
            cv2.putText(image, "Crosswalk detected!",
                        (int(image.shape[1] * 0.05), int(image.shape[0] * 0.4)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        if self.stopline_dist:
            cv2.putText(image, f"Stopline distance: {self.stopline_dist:.2f}",
                        (int(image.shape[1] * 0.05), int(image.shape[0] * 0.2)),
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)
        return image

    def toggle_depth_display(self, show_depth):
        self.show_depth = show_depth

    def sign_callback(self, sign) -> None:
        if sign.data:
            self.numObj = len(sign.data) // 7
            if self.numObj > 0:
                self.detected_objects = np.array(sign.data)  # .reshape(-1, 7).T
        else:
            self.numObj = 0

    def lane_callback(self, lane) -> None:
        self.center = lane.center
        self.crosswalk = lane.crosswalk
        self.stopline = lane.stopline
        self.stopline_dist = lane.stopline_dist
