#!/usr/bin/env python3

import sys
import os
import time
import threading

from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget
from PyQt5.QtGui import QFontDatabase, QFont
from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal, QObject
from python_server.server import Server

from widgets.options import OptionsWidget
from widgets.buttons import ButtonsWidget
from widgets.meters import MeterWidget
from widgets.map import MapWidget
from widgets.camera import CameraWidget
from widgets.message import MessageWidget
from widgets.car import CarWidget

from std_srvs.srv import TriggerRequest


class CommunicationHandler(QObject):
    message_signal = pyqtSignal(str)
    params_signal = pyqtSignal(object, object)
    camera_frame_signal = pyqtSignal(object)
    depth_frame_signal = pyqtSignal(object)
    lane_signal = pyqtSignal(object)
    road_obj_signal = pyqtSignal(object)
    waypoint_signal = pyqtSignal(object)
    sign_signal = pyqtSignal(object)
    run_signal = pyqtSignal(object)


class MainWindow(QMainWindow):
    def __init__(self, server):
        super().__init__()
        self.server = server
        self.comm = CommunicationHandler()

        self.setWindowTitle("BFMC DASHBOARD")

        self.setStyleSheet("""
            background-color: black;
        """)

        self.load_nerd_font()

        self.map_widget = MapWidget(self)
        self.cam_widget = CameraWidget(self)
        self.meter_widget = MeterWidget(self)
        self.car_widget = CarWidget(self)
        self.msg_widget = MessageWidget(self)
        self.buttons_widget = ButtonsWidget(self)
        self.opt_widget = OptionsWidget(self)

        self.comm.message_signal.connect(self.msg_widget.add_message)
        self.comm.params_signal.connect(self.handle_params_update)
        self.comm.camera_frame_signal.connect(self.cam_widget.process_camera_frame)
        self.comm.depth_frame_signal.connect(self.cam_widget.process_depth_frame)
        self.comm.lane_signal.connect(self.cam_widget.lane_callback)
        self.comm.road_obj_signal.connect(self.map_widget.road_objects_callback)
        self.comm.waypoint_signal.connect(self.map_widget.waypoint_callback)
        self.comm.sign_signal.connect(self.handle_sign_update)
        self.comm.run_signal.connect(self.map_widget.call_waypoint_service)

        root_widget = QWidget()
        self.setCentralWidget(root_widget)
        root_layout = QVBoxLayout(root_widget)

        left_widgets = QWidget()
        self.left_layout = QHBoxLayout(left_widgets)
        self.left_layout.setAlignment(QtCore.Qt.AlignJustify)
        self.left_layout.addWidget(self.opt_widget)
        self.left_layout.addWidget(self.map_widget)

        right_widgets = QWidget()
        stat_widgets = QWidget()
        self.right_layout = QVBoxLayout(right_widgets)
        self.right_layout.setAlignment(QtCore.Qt.AlignJustify)
        self.right_layout.addWidget(self.cam_widget)
        self.right_layout.addWidget(self.buttons_widget)
        self.stat_layout = QHBoxLayout(stat_widgets)
        self.stat_layout.setAlignment(QtCore.Qt.AlignJustify)
        self.stat_layout.setSpacing(20)
        self.stat_layout.addWidget(self.meter_widget)
        self.stat_layout.addWidget(self.car_widget)
        self.right_layout.addWidget(stat_widgets)

        top_widgets = QWidget()
        self.top_layout = QHBoxLayout(top_widgets)
        self.top_layout.setAlignment(QtCore.Qt.AlignJustify)
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

    def handle_params_update(self, req, res):
        response = self.map_widget.update_params(req)
        self.server.utility_node_client.send_trigger(TriggerRequest(), response)

    def handle_sign_update(self, sign):
        self.map_widget.sign_callback(sign)
        self.cam_widget.sign_callback(sign)

    def tcp_callbacks(self) -> None:
        while True:
            if self.server.utility_node_client.socket is not None:
                if self.server.utility_node_client.messages:
                    msg = self.server.utility_node_client.messages.popleft()
                    self.comm.message_signal.emit(msg.data)
                if self.server.utility_node_client.triggers.msgs:
                    req, res = self.server.utility_node_client.triggers.msgs.popleft()
                    self.comm.params_signal.emit(req, res)
                if self.server.utility_node_client.run_msg:
                    run = self.server.utility_node_client.run_msg.popleft()
                    self.comm.run_signal.emit(run)
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

            if rgb_image is not None:
                self.comm.camera_frame_signal.emit(rgb_image)
            if depth_image is not None:
                self.comm.depth_frame_signal.emit(depth_image)
            if lane2 is not None:
                self.comm.lane_signal.emit(lane2)
            if road_obj is not None:
                self.comm.road_obj_signal.emit(road_obj)
            if waypoint is not None:
                self.comm.waypoint_signal.emit(waypoint)
            if sign is not None:
                self.comm.sign_signal.emit(sign)

            time.sleep(0.016)


if __name__ == '__main__':
    app = QApplication(sys.argv)

    server = Server()
    server.initialize()

    window = MainWindow(server)
    window.show()

    app.exec()
