#!/usr/bin/env python3

import sys
import os
import time
import threading
import signal

from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget
from PyQt5.QtGui import QFontDatabase, QFont
from PyQt5 import QtCore
from PyQt5.QtCore import pyqtSignal, QObject, Qt
from python_server.server import Server

from widgets.options import OptionsWidget
from widgets.buttons import ButtonsWidget
from widgets.meters import MeterWidget
from widgets.map.map import MapWidget
from widgets.camera import CameraWidget
from widgets.terminal import TerminalWidget
from widgets.car import CarWidget
from widgets.radar import RadarWidget
from widgets.jetson.sw_load import SoftwareMetricsWidget
from widgets.enums import CameraParams

from std_srvs.srv import TriggerRequest


class CommunicationHandler(QObject):
    message_signal = pyqtSignal(str)
    params_signal = pyqtSignal(object, object)
    camera_frame_signal = pyqtSignal(object)
    rgb_frame_signal = pyqtSignal(object)
    depth_frame_signal = pyqtSignal(object)
    lane_signal = pyqtSignal(object)
    road_obj_signal = pyqtSignal(object)
    waypoint_signal = pyqtSignal(object)
    sign_signal = pyqtSignal(object)
    run_signal = pyqtSignal(object)
    steer_signal = pyqtSignal(object)
    sw_load_signal = pyqtSignal(object)
    render_widget_signal = pyqtSignal()


class MainWindow(QMainWindow):
    def __init__(self, server):
        super().__init__()
        signal.signal(signal.SIGINT, self.handle_signal)
        self.alive = True
        self.server = server
        self.comm = CommunicationHandler()

        self.setWindowTitle("BFMC IDE")

        self.setStyleSheet("""
            background-color: black;
        """)

        self.load_nerd_font()
        # self.setWindowFlags(Qt.Window)
        # self.setWindowFlags(Qt.Window | Qt.CustomizeWindowHint | Qt.WindowMinimizeButtonHint | Qt.WindowMaximizeButtonHint | Qt.WindowCloseButtonHint)

        self.map_widget = MapWidget(self)
        self.cam_widget = CameraWidget(self)
        self.meter_widget = MeterWidget(self)
        self.car_widget = CarWidget(self)
        self.terminal_widget = TerminalWidget(self)
        self.buttons_widget = ButtonsWidget(self)
        self.opt_widget = OptionsWidget(self)
        self.radar_widget = RadarWidget(self)
        self.sw_widget = SoftwareMetricsWidget(self)

        self.comm.message_signal.connect(self.terminal_widget.add_message)
        self.comm.params_signal.connect(self.handle_params_update)
        self.comm.camera_frame_signal.connect(self.cam_widget.process_camera_frame)
        self.comm.rgb_frame_signal.connect(self.buttons_widget.save_frame)
        self.comm.depth_frame_signal.connect(self.cam_widget.process_depth_frame)
        self.comm.lane_signal.connect(self.cam_widget.lane_callback)
        self.comm.road_obj_signal.connect(self.map_widget.road_objects_callback)
        self.comm.waypoint_signal.connect(self.map_widget.waypoint_callback)
        self.comm.sign_signal.connect(self.handle_sign_update)
        self.comm.run_signal.connect(self.map_widget.call_waypoint_service)
        self.comm.steer_signal.connect(self.car_widget.set_steer)
        self.comm.steer_signal.connect(self.meter_widget.set_steer)
        self.comm.sw_load_signal.connect(self.sw_widget.set_load)

        self.comm.render_widget_signal.connect(self.car_widget.render_widget)
        self.comm.render_widget_signal.connect(self.meter_widget.render_widget)
        self.comm.render_widget_signal.connect(self.radar_widget.render_widget)
        self.comm.render_widget_signal.connect(self.sw_widget.render_widget)

        root_widget = QWidget()
        self.setCentralWidget(root_widget)
        root_layout = QHBoxLayout(root_widget)
        root_layout.setContentsMargins(10, 10, 10, 10)

        left_widgets = QWidget()
        self.left_layout = QVBoxLayout(left_widgets)
        self.left_layout.setContentsMargins(0, 0, 0, 0)

        top_widgets = QWidget()
        self.top_layout = QHBoxLayout(top_widgets)
        self.top_layout.setContentsMargins(0, 0, 0, 0)
        self.top_layout.addWidget(self.opt_widget)
        self.top_layout.addWidget(self.map_widget)

        self.left_layout.addWidget(top_widgets, 5)
        self.left_layout.addWidget(self.terminal_widget, 2)

        right_widgets = QWidget()
        stat_widgets = QWidget()
        cam_wrapper = QWidget()
        self.cam_wrapper_layout = QVBoxLayout(cam_wrapper)
        self.cam_wrapper_layout.setContentsMargins(0, 0, 0, 0)
        self.cam_wrapper_layout.addWidget(self.cam_widget)
        self.cam_wrapper_layout.addWidget(self.buttons_widget)
        self.right_layout = QVBoxLayout(right_widgets)
        self.right_layout.setContentsMargins(0, 0, 0, 0)
        self.right_layout.addWidget(cam_wrapper, 2)

        self.stat_layout = QVBoxLayout(stat_widgets)
        self.stat_layout.setContentsMargins(0, 0, 0, 0)
        self.stat_layout.setAlignment(QtCore.Qt.AlignCenter)
        left_wrapper = QWidget()
        right_wrapper = QWidget()
        self.left_wrapper_layout = QHBoxLayout(left_wrapper)
        self.left_wrapper_layout.setContentsMargins(0, 0, 0, 0)
        self.left_wrapper_layout.setAlignment(QtCore.Qt.AlignTop)
        meter_wrapper = QWidget()
        meter_layout = QHBoxLayout(meter_wrapper)
        meter_layout.setAlignment(QtCore.Qt.AlignVCenter)
        meter_layout.setContentsMargins(0, 0, 0, 0)
        meter_layout.addWidget(self.meter_widget)
        self.left_wrapper_layout.addWidget(meter_wrapper)
        radar_wrapper = QWidget()
        radar_layout = QHBoxLayout(radar_wrapper)
        radar_wrapper.setContentsMargins(5, 0, 5, 5)
        radar_layout.addWidget(self.radar_widget)
        self.left_wrapper_layout.addWidget(radar_wrapper)
        self.right_wrapper_layout = QHBoxLayout(right_wrapper)
        self.right_wrapper_layout.setContentsMargins(0, 0, 0, 0)
        self.right_wrapper_layout.setAlignment(QtCore.Qt.AlignTop)
        self.right_wrapper_layout.addWidget(self.sw_widget)
        self.right_wrapper_layout.addWidget(self.car_widget)
        self.stat_layout.addWidget(left_wrapper)
        self.stat_layout.addWidget(right_wrapper)
        self.right_layout.addWidget(stat_widgets, 1)

        root_layout.addWidget(left_widgets, 2)
        root_layout.addWidget(right_widgets, 1)

        self.terminal_widget.add_message("BFMC IDE INITIALIZED")

        self.udp_thread = threading.Thread(target=self.udp_callbacks, args=(), daemon=True)
        self.tcp_thread = threading.Thread(target=self.tcp_callbacks, args=(), daemon=True)
        self.cam_thread = threading.Thread(target=self.cam_record_callback, args=(), daemon=True)
        self.udp_thread.start()
        self.tcp_thread.start()
        self.cam_thread.start()

        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(32)
        self.timer.timeout.connect(self.render_callbacks)
        self.timer.start()

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
        while self.alive:
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
            time.sleep(CameraParams.FPS_30.value)

    def udp_callbacks(self) -> None:
        while self.alive:
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
            steer = self.server.udp_connection.parse_steer()
            load = self.server.udp_connection.parse_sw_load()

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
            if steer is not None:
                self.comm.steer_signal.emit(steer)
            if load is not None:
                self.comm.sw_load_signal.emit(load)
            time.sleep(CameraParams.FPS_60.value)

    def cam_record_callback(self) -> None:
        while self.alive:
            if self.buttons_widget.recording:
                rgb_image = self.server.udp_connection.parse_rgb_image()
                if rgb_image is not None:
                    self.comm.rgb_frame_signal.emit(rgb_image)
                time.sleep(CameraParams.RECORDING_REFRESH_RATE.value)
            time.sleep(CameraParams.RECORDING_REFRESH_RATE.value)

    def render_callbacks(self) -> None:
        self.comm.render_widget_signal.emit()

    def closeEvent(self, event):
        try:
            self.alive = False
            self.car_widget.cleanup_gl_resources()
            self.terminal_widget.terminate_processes()
            time.sleep(0.2)
            print("Processes terminated")
            if self.server.tcp_socket:
                self.server.tcp_socket.close()
                self.server.udp_socket.close()
            print("Sockets closed")
        except Exception:
            pass
        event.accept()

    def handle_signal(self, signal, frame):
        print("Caught SIGINT (Ctrl+C), closing application...")
        self.close()
        sys.exit(0)


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    app = QApplication(sys.argv)
    server = Server()
    server.initialize()

    window = MainWindow(server)
    window.show()

    app.exec()
