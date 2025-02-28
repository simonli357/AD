#!/usr/bin/env python3

import sys
import os
import time
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget
from PyQt5.QtGui import QFontDatabase, QFont
from python_server.server import Server

from widgets.options import OptionsWidget
from widgets.buttons import ButtonsWidget
from widgets.meters import MeterWidget
from widgets.map import MapWidget
from widgets.camera import CameraWidget
from widgets.message import MessageWidget

from std_srvs.srv import TriggerRequest


class MainWindow(QMainWindow):
    def __init__(self, server):
        super().__init__()
        self.server = server

        self.setWindowTitle("BFMC DASHBOARD")
        self.setStyleSheet("background-color: black;")
        self.load_nerd_font()

        self.opt_widget = OptionsWidget()
        self.map_widget = MapWidget()
        self.cam_widget = CameraWidget()
        self.buttons_widget = ButtonsWidget()
        self.meter_widget = MeterWidget()
        self.msg_widget = MessageWidget()

        root_widget = QWidget()
        self.setCentralWidget(root_widget)
        root_layout = QVBoxLayout(root_widget)

        left_widgets = QWidget()
        self.left_layout = QHBoxLayout(left_widgets)
        self.left_layout.addWidget(self.opt_widget)
        self.left_layout.addWidget(self.map_widget)

        right_widgets = QWidget()
        self.right_layout = QVBoxLayout(right_widgets)
        self.right_layout.addWidget(self.cam_widget)
        self.right_layout.addWidget(self.buttons_widget)
        self.right_layout.addWidget(self.meter_widget)

        top_widgets = QWidget()
        self.top_layout = QHBoxLayout(top_widgets)
        self.top_layout.addWidget(left_widgets)
        self.top_layout.addWidget(right_widgets)

        root_layout.addWidget(top_widgets)
        root_layout.addWidget(self.msg_widget)

        self.msg_widget.add_message("BFMC DASHBOARD INITIALIZED")

        threading.Thread(target=self.udp_callbacks, args=(), daemon=True).start()
        threading.Thread(target=self.tcp_callbacks, args=(), daemon=True).start()

    def load_nerd_font(self) -> None:
        current_dir = os.path.dirname(os.path.abspath(__file__))
        font_path = os.path.join(current_dir, 'fonts/HackNerdFont-Bold.ttf')
        font_id = QFontDatabase.addApplicationFont(font_path)
        if font_id == -1:
            print("Error: Failed to load Nerd Font!")
            sys.exit(1)
        font_family = QFontDatabase.applicationFontFamilies(font_id)[0]
        nerd_font = QFont(font_family, 10)
        QApplication.setFont(nerd_font)
        print(f"Successfully loaded font: {font_family}")

    def tcp_callbacks(self) -> None:
        while True:
            if self.server.utility_node_client.socket is not None:
                # Messages
                if self.server.utility_node_client.messages:
                    self.msg_widget.add_message(self.server.utility_node_client.messages.popleft())
                # Set params
                if self.server.utility_node_client.triggers.msgs:
                    req, res = self.server.utility_node_client.triggers.msgs.popleft()
                    response = self.map_widget.update_params(req)
                    self.server.utility_node_client.send_trigger(TriggerRequest(), response)
            time.sleep(0.016)

    def udp_callbacks(self) -> None:
        while True:
            rgb_image = None
            depth_image = None
            if self.cam_widget.show_depth:
                depth_image = self.server.udp_connection.parse_depth_image()
            else:
                rgb_image = self.server.udp_connection.parse_rgb_image()
            sign = self.server.udp_connection.parse_sign()
            waypoint = self.server.udp_connection.parse_waypoint()
            road_obj = self.server.udp_connection.parse_road_object()
            lane2 = self.server.udp_connection.parse_lane2()
            # Image rgb
            if rgb_image is not None:
                self.cam_widget.process_camera_frame(rgb_image)
            # Image depth
            if depth_image is not None:
                self.cam_widget.process_depth_frame(depth_image)
            # Lane2
            if lane2 is not None:
                self.map_widget.lane_callback(lane2)
            # Road object
            if road_obj is not None:
                self.map_widget.road_objects_callback(road_obj)
            # Waypoints
            if waypoint is not None:
                self.map_widget.waypoint_callback(waypoint)
            # Signs
            if sign is not None:
                self.map_widget.sign_callback(sign)
            time.sleep(0.016)


if __name__ == '__main__':
    app = QApplication(sys.argv)

    server = Server()
    server.initialize()

    window = MainWindow(server)
    window.show()

    app.exec()
