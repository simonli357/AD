from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import pyqtSignal
from cv_bridge import CvBridge

import cv2
import numpy as np


class CameraWidget(QtWidgets.QWidget):
    update_camera_signal = pyqtSignal(QtGui.QPixmap)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.show_depth = False
        self.bridge = CvBridge()
        self.setup_ui()
        self.update_camera_signal.connect(self.update_camera_display)

    def setup_ui(self):
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

        # Camera display label
        self.camera_label = QtWidgets.QLabel(self)
        self.camera_label.setFixedSize(640, 480)
        self.camera_label.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addWidget(self.camera_label)

        # Styling
        self.setStyleSheet("""
            background-color: rgba(255, 255, 255, 0.05);
        """)

    def update_camera_display(self, pixmap):
        self.camera_label.setPixmap(pixmap)

    def process_camera_frame(self, frame):
        """Process RGB camera frame"""
        try:
            if frame is None or frame.size == 0:
                return

            # Add detection overlays
            frame = self.add_sign_detection_to_image(frame)
            frame = self.add_lane_detection_to_image(frame)

            # Convert and resize
            rgb_image = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            rgb_image = cv2.resize(rgb_image,
                                   (self.camera_label.width(),
                                    self.camera_label.height()))

            # Convert to QPixmap
            h, w, ch = rgb_image.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(rgb_image.data, w, h, bytes_per_line,
                                    QtGui.QImage.Format_RGB888)
            self.update_camera_signal.emit(QtGui.QPixmap.fromImage(qt_image))

        except Exception as e:
            print(f"Camera processing error: {e}")

    def process_depth_frame(self, depth_frame):
        """Process depth camera frame"""
        try:
            depth_normalized = cv2.normalize(depth_frame, None, 50, 255, cv2.NORM_MINMAX)
            depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8),
                                              cv2.COLORMAP_TURBO)
            depth_colored = cv2.resize(depth_colored,
                                       (self.camera_label.width(),
                                        self.camera_label.height()))

            # Convert to QPixmap
            h, w, ch = depth_colored.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(depth_colored.data, w, h, bytes_per_line,
                                    QtGui.QImage.Format_RGB888)
            self.update_camera_signal.emit(QtGui.QPixmap.fromImage(qt_image))

        except Exception as e:
            print(f"Depth processing error: {e}")

    def add_sign_detection_to_image(self, image):
        return image

    def add_lane_detection_to_image(self, image):
        return image

    def toggle_depth_display(self, show_depth):
        self.show_depth = show_depth
