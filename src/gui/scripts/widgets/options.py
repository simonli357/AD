from PyQt5 import QtWidgets, QtCore
from collections import deque


class OptionsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.buttons = deque()

        # Create buttons
        self.toggle_sign_btn = QtWidgets.QPushButton("󰞁")
        self.toggle_lanes_btn = QtWidgets.QPushButton("󰑢")
        self.toggle_cars_btn = QtWidgets.QPushButton("󰭮")
        self.toggle_destinations_btn = QtWidgets.QPushButton("󰍐")
        self.toggle_path_btn = QtWidgets.QPushButton("")
        self.toggle_gt_btn = QtWidgets.QPushButton("󰨚")
        self.toggle_depth_btn = QtWidgets.QPushButton("󰤽")
        self.set_states_btn = QtWidgets.QPushButton("󰵉")
        self.set_yaw_btn = QtWidgets.QPushButton("󰆋")
        self.save_path_btn = QtWidgets.QPushButton("󰆓")

        self.buttons.append(self.toggle_sign_btn)
        self.buttons.append(self.toggle_lanes_btn)
        self.buttons.append(self.toggle_cars_btn)
        self.buttons.append(self.toggle_destinations_btn)
        self.buttons.append(self.toggle_path_btn)
        self.buttons.append(self.toggle_gt_btn)
        self.buttons.append(self.toggle_depth_btn)
        self.buttons.append(self.set_states_btn)
        self.buttons.append(self.set_yaw_btn)
        self.buttons.append(self.save_path_btn)

        # Add buttons to layout with spacing
        for btn in self.buttons:
            self.layout.addWidget(btn)

        # Add bottom stretch to keep buttons at top
        self.layout.addStretch()

        # Styling
        self.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 255, 255, 0.15);
                padding: 12px 18px 12px 12px;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 32px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #1c6da8;
            }
        """)
