from PyQt5 import QtWidgets, QtCore
from ..enums import CameraParams
from .hud import CameraOverlay

import numpy as np


class CameraWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.main_window = self.parent()
        self.show_depth = False
        self.numObj = 0
        self.detected_objects = np.zeros(7)
        self.emergency = False
        self.setup_ui()

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

    def process_camera_frame(self, pixmap):
        try:
            if self.show_depth:
                return
            self.camera_label.setPixmap(pixmap)
        except Exception as e:
            print(f"Camera processing error: {e}")

    def process_depth_frame(self, pixmap):
        try:
            if not self.show_depth:
                return
            self.camera_label.setPixmap(pixmap)
        except Exception as e:
            print(f"Depth processing error: {e}")

    def toggle_depth_display(self, show_depth):
        self.show_depth = show_depth

    def sign_callback(self, sign) -> None:
        if sign.data:
            data = np.array(sign.data)
            data = data[:(len(data) // 7) * 7].reshape(-1, 7)
            sentinel_mask = (data[:, 5] == -1) & (data[:, 6] == -1)
            self.emergency = bool(np.any(sentinel_mask))
            data = data[~sentinel_mask]
            self.numObj = len(data)
            self.detected_objects = data.reshape(-1) if self.numObj > 0 else np.array([])
        else:
            self.numObj = 0
            self.detected_objects = np.array([])
            self.emergency = False

    def lane_callback(self, lane) -> None:
        self.hud.center = lane.center
        self.hud.crosswalk = lane.crosswalk
        self.hud.stopline = lane.stopline
        self.hud.stopline_dist = lane.stopline_dist
