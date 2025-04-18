from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QLabel, QWidget, QApplication
from ..enums import MapData

import numpy as np


class HidableOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)
        self.setAttribute(QtCore.Qt.WA_StyledBackground, True)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setStyleSheet("""
            background-color: transparent;
            border: none;
            border-radius: 8px;
        """)
        self.car_widget = self.parent()
        self.global_rect = QtCore.QRect()
        self.event_filter_installed = False

        self.destinations = None
        self.path = []
        self.visited = set()
        self.dist_traveled = 0

        self.setup_ui()
        self.update_global_rect()

    def setup_ui(self):
        self.layout = QtWidgets.QVBoxLayout()
        self.total_dist_label = QLabel('󰣰  Distance: --:--')
        self.total_dist_label.setStyleSheet("""
            border: none;
            padding: 2px;
            background-color: transparent;
            color: yellow;
            font-size: 12px;
        """)
        self.dist_traveled_label = QLabel('  Traveled: --:--')
        self.dist_traveled_label.setStyleSheet("""
            border: none;
            padding: 2px;
            background-color: transparent;
            color: yellow;
            font-size: 12px;
        """)
        self.dest_reached_label = QLabel('󰪥  Reached: --:--')
        self.dest_reached_label.setStyleSheet("""
            border: none;
            padding: 2px;
            background-color: transparent;
            color: yellow;
            font-size: 12px;
        """)
        self.car_pose_label = QLabel('󰵉  Pose: --:--')
        self.car_pose_label.setStyleSheet("""
            border: none;
            padding: 2px;
            background-color: transparent;
            color: yellow;
            font-size: 12px;
        """)
        self.car_rotation_label = QLabel('󰵗  Rotation: --:--')
        self.car_rotation_label.setStyleSheet("""
            border: none;
            padding: 2px;
            background-color: transparent;
            color: yellow;
            font-size: 12px;
        """)

        self.layout.addWidget(self.total_dist_label)
        self.layout.addWidget(self.dist_traveled_label)
        self.layout.addWidget(self.dest_reached_label)
        self.layout.addWidget(self.car_pose_label)
        self.layout.addWidget(self.car_rotation_label)
        self.setLayout(self.layout)

    def calculate_total_path_distance(self):
        if self.car_widget.main_window.state_refs_np.shape[1] < 2:
            return 0.0
        x_coords = self.car_widget.main_window.state_refs_np[0, :]
        y_coords = self.car_widget.main_window.state_refs_np[1, :]
        dx = x_coords[1:] - x_coords[:-1]
        dy = y_coords[1:] - y_coords[:-1]
        distances = np.sqrt(dx**2 + dy**2)
        return np.sum(distances)

    def is_near(self, x1: float, y1: float, x2: float, y2: float, rad1: float, rad2: float):
        return (x2 - x1)**2 + (y2 - y1)**2 <= (rad1 + rad2)**2

    def update_visited_destinations(self, car_x: float, car_y: float):
        for idx, row in self.car_widget.main_window.destinations.iterrows():
            x = row['X']
            y = row['Y']
            y_inverted = MapData.REAL_WORLD_HEIGHT.value - row['Y']
            if self.is_near(car_x, car_y, x, y_inverted, 0.1, 0.1):
                self.car_widget.main_window.visited.add((x, y))
                self.set_dest_visited_num(len(self.visited))
                self.car_widget.update_visited_destination(x, y_inverted)
                break

    def set_total_path_distance(self):
        dist = self.calculate_total_path_distance()
        self.total_dist_label.setText(f'󰣰  Distance: {dist:.2f}')

    def set_distance_traveled(self) -> None:
        self.dist_traveled_label.setText(f'  Traveled: {self.dist_traveled:.2f}')

    def set_dest_visited_num(self, dest_visited: int) -> None:
        self.dest_reached_label.setText(f'󰪥  Reached: {dest_visited:.0f}')

    def set_car_pose(self, x, y, z):
        self.car_pose_label.setText(f"󰵉  x:{x:.2f} y:{(13.786 - y):.2f} z:{z:.1f}")

    def set_car_rotation(self, yaw, steer):
        self.car_rotation_label.setText(f"󰵗  yaw:{yaw:.2f} steer:{steer:.2f}")

    def update_global_rect(self):
        if self.parent():
            parent_global = self.parent().mapToGlobal(QtCore.QPoint(0, 0))
            self_global = parent_global + self.pos()
            self.global_rect = QtCore.QRect(self_global, self.size())
        else:
            self.global_rect = QtCore.QRect(
                self.mapToGlobal(QtCore.QPoint(0, 0)),
                self.size()
            )

    ################
    # Events
    ################

    def resizeEvent(self, event):
        self.update_global_rect()
        super().resizeEvent(event)

    def moveEvent(self, event):
        self.update_global_rect()
        super().moveEvent(event)

    def enterEvent(self, event):
        self.hide()
        if not self.event_filter_installed:
            QApplication.instance().installEventFilter(self)
            self.event_filter_installed = True
        super().enterEvent(event)

    def eventFilter(self, obj, event):
        if event.type() == QtCore.QEvent.MouseMove:
            self.update_global_rect()
            pos = event.globalPos()
            if not self.global_rect.contains(pos):
                self.show()
                QApplication.instance().removeEventFilter(self)
                self.event_filter_installed = False
        return super().eventFilter(obj, event)
