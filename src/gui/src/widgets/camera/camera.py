from PyQt5 import QtWidgets, QtGui, QtCore
from PyQt5.QtCore import pyqtSignal
from ..enums import CameraParams
from .hud import CameraOverlay

import numpy as np


class CameraWidget(QtWidgets.QWidget):
    update_camera_signal = pyqtSignal(QtGui.QPixmap)

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.main_window = self.parent()
        self.has_frame = False
        self.show_depth = False
        self.numObj = 0
        self.detected_objects = np.zeros(7)

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

    def process_camera_frame(self, pixmap):
        try:
            if self.show_depth:
                return
            self.update_camera_signal.emit(pixmap)
        except Exception as e:
            print(f"Camera processing error: {e}")

    def process_depth_frame(self, pixmap):
        try:
            if not self.show_depth:
                return
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
        self.hud.center = lane.center
        self.hud.crosswalk = lane.crosswalk
        self.hud.stopline = lane.stopline
        self.hud.stopline_dist = lane.stopline_dist
