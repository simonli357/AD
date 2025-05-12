from PyQt5 import QtWidgets, QtCore
from PyQt5.QtGui import QCursor
from collections import deque

import os
import numpy as np


class SidebarWidget(QtWidgets.QWidget):
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
        self.layout.setContentsMargins(0, 0, 0, 0)
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.buttons = deque()

        self.toggle_sign_btn = QtWidgets.QPushButton("󰇐")
        self.toggle_lanes_btn = QtWidgets.QPushButton("󰑢")
        self.toggle_cars_btn = QtWidgets.QPushButton("󰭮")
        self.toggle_destinations_btn = QtWidgets.QPushButton("󰍐")
        self.toggle_path_btn = QtWidgets.QPushButton("󰴠")
        self.toggle_gt_btn = QtWidgets.QPushButton("󰒕")
        self.toggle_depth_btn = QtWidgets.QPushButton("󰤽")
        self.toggle_graph_editor_btn = QtWidgets.QPushButton("")
        self.fetch_run_btn = QtWidgets.QPushButton("󱑤")
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
        self.buttons.append(self.toggle_graph_editor_btn)
        self.buttons.append(self.fetch_run_btn)
        self.buttons.append(self.set_states_btn)
        self.buttons.append(self.set_yaw_btn)
        self.buttons.append(self.save_path_btn)

        self.toggle_sign_btn.setToolTip("Toggle Signs")
        self.toggle_lanes_btn.setToolTip("Toggle Lanes")
        self.toggle_cars_btn.setToolTip("Toggle Cars")
        self.toggle_destinations_btn.setToolTip("Toggle Destinations")
        self.toggle_path_btn.setToolTip("Toggle Path")
        self.toggle_gt_btn.setToolTip("Toggle Ground Truth")
        self.toggle_depth_btn.setToolTip("Toggle Depth")
        self.toggle_graph_editor_btn.setToolTip("Toggle Graph Editor")
        self.fetch_run_btn.setToolTip("Fetch Simulator Run")
        self.set_states_btn.setToolTip("Set States")
        self.set_yaw_btn.setToolTip("Set Yaw")
        self.save_path_btn.setToolTip("Save Path")

        for btn in self.buttons:
            self.layout.addWidget(btn)

        self.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 255, 255, 0.08);
                padding: 12px 18px 12px 12px;
                color: white;
                border: none;
                border-radius: 8px;
                font-size: 24px;
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

        self.initialize_buttons()

    def initialize_buttons(self) -> None:
        self.update_button_style(self.toggle_sign_btn, self.map_widget.show_signs)
        self.update_button_style(self.toggle_lanes_btn, self.map_widget.show_lanes)
        self.update_button_style(self.toggle_cars_btn, self.map_widget.show_cars)
        self.update_button_style(self.toggle_destinations_btn, self.map_widget.show_destinations)
        self.update_button_style(self.toggle_path_btn, self.map_widget.show_path)
        self.update_button_style(self.toggle_gt_btn, self.map_widget.show_gt)
        self.update_button_style(self.toggle_depth_btn, self.cam_widget.show_depth)
        self.update_button_style(self.toggle_depth_btn, self.map_widget.show_graph)

    def connect_signals(self) -> None:
        self.toggle_sign_btn.clicked.connect(self.handle_sign_btn_click)
        self.toggle_lanes_btn.clicked.connect(self.handle_lanes_btn_click)
        self.toggle_cars_btn.clicked.connect(self.handle_cars_btn_click)
        self.toggle_destinations_btn.clicked.connect(self.handle_destinations_btn_click)
        self.toggle_path_btn.clicked.connect(self.handle_path_btn_click)
        self.toggle_gt_btn.clicked.connect(self.handle_gt_btn_click)
        self.toggle_depth_btn.clicked.connect(self.handle_depth_btn_click)
        self.toggle_graph_editor_btn.clicked.connect(self.handle_graph_btn_click)
        self.set_states_btn.clicked.connect(self.handle_states_btn_click)
        self.set_yaw_btn.clicked.connect(self.handle_yaw_btn_click)
        self.save_path_btn.clicked.connect(self.handle_save_path_btn_click)
        self.fetch_run_btn.clicked.connect(self.handle_fetch_run_btn_click)

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

    def handle_graph_btn_click(self) -> None:
        self.map_widget.show_graph = not self.map_widget.show_graph
        self.map_widget.graph_editor.G = self.main_window.database.graph_queries.fetch_graph()
        self.update_button_style(self.toggle_graph_editor_btn, self.map_widget.show_graph)

    def handle_states_btn_click(self) -> None:
        if not self.map_widget.show_nodes:
            self.call_set_states_service(self.map_widget.cursor_x, self.map_widget.cursor_y)
        else:
            x, y = self.map_widget.graphics_view.get_start()
            self.call_set_states_service(x, y)

    def handle_yaw_btn_click(self) -> None:
        menu = QtWidgets.QMenu(self.map_widget)
        action_0 = menu.addAction("NORTH 0° ")
        action_1 = menu.addAction("EAST 90° ")
        action_2 = menu.addAction("SOUTH 180° ")
        action_3 = menu.addAction("WEST 270° ")
        menu.setStyleSheet("""
            QMenu {
                color: white;
                font-size: 16px;
                border: none;
                background-color: transparent;
            }
            QMenu::item {
                background-color: rgba(40, 40, 40, 0.5);
                margin: 1px;
                padding-left: 30px;
                padding-right: 30px;
                padding-top: 4px;
                padding-bottom: 4px;
                border-radius: 8px;
            }
            QMenu::item:selected {
                background-color: purple;
            }
        """)
        chosen = menu.exec(QCursor.pos())
        if chosen == action_0:
            self.call_set_yaw_service(0)
        elif chosen == action_1:
            self.call_set_yaw_service(1)
        elif chosen == action_2:
            self.call_set_yaw_service(2)
        elif chosen == action_3:
            self.call_set_yaw_service(3)

    def handle_save_path_btn_click(self) -> None:
        path = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))
        path = os.path.join(path, 'saved')
        os.makedirs(path, exist_ok=True)
        if self.main_window.map_widget.show_graph:
            self.main_window.map_widget.graph_editor.export(os.path.join(path, 'graph.graphml'))
        else:
            np.savetxt(os.path.join(path, 'state_refs1.txt'), self.main_window.state_refs_np.T, fmt='%.4f')
            print("saved state refs")

    def handle_fetch_run_btn_click(self) -> None:
        try:
            self.main_window.server.tcp_client.send_string('refresh_run')
        except Exception:
            pass

    def on_set_states(self, success):
        self.main_window.reset_run_statistics()

    def call_set_states_service(self, x=-200.0, y=-200.0):
        try:
            if self.server.tcp_client is None:
                print("tcp_client not online")
                return
            print("set states service called")
            self.server.tcp_client.send_set_states_srv(x, y)
        except Exception as e:
            print(e)

    def call_set_yaw_service(self, direction):
        try:
            if self.server.tcp_client is None:
                print("tcp_client not online")
                return
            print("set yaw service called")
            self.server.tcp_client.send_yaw(direction)
        except Exception as e:
            print(e)
