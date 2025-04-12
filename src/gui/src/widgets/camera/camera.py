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
        self.center = 320
        self.crosswalk = False
        self.stopline = False
        self.stopline_dist = None

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
        self.hud.update_overlay()

    def update_hud(self):
        if not self.has_frame:
            self.hud.update_overlay()

    def process_camera_frame(self, cv_image):
        """Process RGB camera frame"""
        try:
            if self.show_depth:
                return

            # cv_image = self.add_sign_detection_to_image(cv_image)
            # cv_image = self.add_lane_detection_to_image(cv_image)
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
            # depth_colored = self.add_sign_detection_to_image(depth_colored)
            # depth_colored = self.add_lane_detection_to_image(depth_colored)

            # Convert to QImage
            h, w, ch = depth_colored.shape
            bytes_per_line = ch * w
            qt_image = QtGui.QImage(depth_colored.data, w, h, bytes_per_line, QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(qt_image)
            self.update_camera_signal.emit(pixmap)
        except Exception as e:
            print(f"Depth processing error: {e}")

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
