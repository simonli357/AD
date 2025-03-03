from PyQt5 import QtWidgets, QtCore
from collections import deque

import time
import os
import numpy as np


class OptionsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.server = self.main_window.server
        self.map_widget = self.main_window.map_widget
        self.cam_widget = self.main_window.cam_widget
        self.setup_ui()
        self.connect_signals()

    def setup_ui(self) -> None:
        self.window().setAttribute(QtCore.Qt.WA_AlwaysShowToolTips, True)
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

        self.toggle_sign_btn.setToolTip("Toggle Signs")
        self.toggle_lanes_btn.setToolTip("Toggle Lanes")
        self.toggle_cars_btn.setToolTip("Toggle Cars")
        self.toggle_destinations_btn.setToolTip("Toggle Destinations")
        self.toggle_path_btn.setToolTip("Toggle Path")
        self.toggle_gt_btn.setToolTip("Toggle Objects")
        self.toggle_depth_btn.setToolTip("Toggle Depth")
        self.set_states_btn.setToolTip("Set States")
        self.set_yaw_btn.setToolTip("Set Yaw")
        self.save_path_btn.setToolTip("Save Path")

        # Add buttons to layout with spacing
        for btn in self.buttons:
            self.layout.addWidget(btn)

        # Styling
        self.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 255, 255, 0.08);
                padding: 12px 18px 12px 12px;
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 32px;
            }
            QPushButton:hover {
                background-color: #9933ff;
            }
            QPushButton:pressed {
                background-color: #cc99ff;
            }
            QToolTip {
                background-color: black;
                color: white;
                border: none;
                font-size: 14px;
                padding: 5px;
            }
        """)

        self.update_button_style(self.toggle_sign_btn, self.map_widget.show_signs)
        self.update_button_style(self.toggle_lanes_btn, self.map_widget.show_lanes)
        self.update_button_style(self.toggle_cars_btn, self.map_widget.show_cars)
        self.update_button_style(self.toggle_destinations_btn, self.map_widget.show_destinations)
        self.update_button_style(self.toggle_path_btn, self.map_widget.show_path)
        self.update_button_style(self.toggle_gt_btn, self.map_widget.show_gt)
        self.update_button_style(self.toggle_depth_btn, self.cam_widget.show_depth)

    def call_set_states_service(self, x=None, y=None):
        print("set states service called")
        try:
            if self.server.utility_node_client.socket is None:
                return
            if x is not None and y is not None:
                self.server.utility_node_client.send_set_states_srv(x, y)
            else:
                self.server.utility_node_client.send_set_states_srv(-200.0, -200.0)
            max_retries = 50
            retries = 0
            while (retries < max_retries):
                if self.server.utility_node_client.set_states_srv_msg.success:
                    print("Successful set_states service call")
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to set states")
        except Exception as e:
            print(e)

    def connect_signals(self) -> None:
        self.toggle_sign_btn.clicked.connect(self.handle_sign_btn_click)
        self.toggle_lanes_btn.clicked.connect(self.handle_lanes_btn_click)
        self.toggle_cars_btn.clicked.connect(self.handle_cars_btn_click)
        self.toggle_destinations_btn.clicked.connect(self.handle_destinations_btn_click)
        self.toggle_path_btn.clicked.connect(self.handle_path_btn_click)
        self.toggle_gt_btn.clicked.connect(self.handle_gt_btn_click)
        self.toggle_depth_btn.clicked.connect(self.handle_depth_btn_click)
        self.set_states_btn.clicked.connect(self.handle_states_btn_click)
        self.set_yaw_btn.clicked.connect(self.handle_yaw_btn_click)
        self.save_path_btn.clicked.connect(self.handle_save_path_btn_click)

    def update_button_style(self, button, is_active):
        """Update button color based on boolean state"""
        color = "#9933FF" if is_active else "rgba(255, 255, 255, 0.08);"  # Light green/red
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
            }}
            QPushButton:hover {{
                background-color: #9933ff;
            }}
        """)

    def handle_sign_btn_click(self) -> None:
        self.map_widget.show_signs = not self.map_widget.show_signs
        self.update_button_style(self.toggle_sign_btn, self.map_widget.show_signs)

    def handle_lanes_btn_click(self) -> None:
        self.map_widget.show_lanes = not self.map_widget.show_lanes
        self.update_button_style(self.toggle_lanes_btn, self.map_widget.show_lanes)

    def handle_cars_btn_click(self) -> None:
        self.map_widget.show_cars = not self.map_widget.show_cars
        self.update_button_style(self.toggle_cars_btn, self.map_widget.show_cars)

    def handle_destinations_btn_click(self) -> None:
        self.map_widget.show_destinations = not self.map_widget.show_destinations
        self.update_button_style(self.toggle_destinations_btn, self.map_widget.show_destinations)

    def handle_path_btn_click(self) -> None:
        self.map_widget.show_path = not self.map_widget.show_path
        self.update_button_style(self.toggle_path_btn, self.map_widget.show_path)

    def handle_gt_btn_click(self) -> None:
        self.map_widget.show_gt = not self.map_widget.show_gt
        self.update_button_style(self.toggle_gt_btn, self.map_widget.show_gt)

    def handle_depth_btn_click(self) -> None:
        self.cam_widget.show_depth = not self.cam_widget.show_depth
        self.update_button_style(self.toggle_depth_btn, self.cam_widget.show_depth)

    def handle_states_btn_click(self) -> None:
        self.call_set_states_service(self.map_widget.cursor_x, self.map_widget.cursor_y)

    def handle_yaw_btn_click(self) -> None:
        self.call_set_states_service()

    def handle_save_path_btn_click(self) -> None:
        path = os.path.dirname(os.path.abspath(__file__))
        np.savetxt(os.path.join(path, 'state_refs1.txt'), self.state_refs_np.T, fmt='%.4f')
        print("saved state refs")
