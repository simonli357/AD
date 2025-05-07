#!/usr/bin/env python3

import sys
import os
import time
import threading
import signal
import argparse

from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget
from PyQt5.QtGui import QFontDatabase, QFont
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal, QObject, Qt, QTimer
from python_server.server import Server
from database.database import Database
from widgets.sidebar.sidebar import SidebarWidget
from widgets.camera.camera import CameraWidget
from widgets.camera.buttons import ButtonsWidget
from widgets.map.map import MapWidget
from widgets.buttons_overlay import ButtonsOverlay
from widgets.barca.barca import BarcaWidget
from widgets.car.car import CarWidget
from widgets.terminal.terminal import TerminalWidget
from widgets.enums import CameraParams


class CommunicationHandler(QObject):
    start_signal = pyqtSignal()
    message_signal = pyqtSignal(str)
    waypoints_signal = pyqtSignal(object)
    params_signal = pyqtSignal(object)
    run_signal = pyqtSignal(object)
    goto_signal = pyqtSignal(object)
    set_states_signal = pyqtSignal(object)

    camera_frame_signal = pyqtSignal(QtGui.QPixmap)
    depth_frame_signal = pyqtSignal(QtGui.QPixmap)
    depth_arr_signal = pyqtSignal(object)
    lane_signal = pyqtSignal(object)
    road_obj_signal = pyqtSignal(object)
    waypoint_signal = pyqtSignal(object)
    sign_signal = pyqtSignal(object)
    steer_signal = pyqtSignal(object)
    sw_load_signal = pyqtSignal(object)


class MapContainer(QtWidgets.QStackedWidget):
    mouse_left = pyqtSignal()
    mouse_enter = pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMouseTracking(True)

    def enterEvent(self, event):
        self.mouse_enter.emit()
        super().enterEvent(event)

    def leaveEvent(self, event):
        self.mouse_left.emit()
        super().leaveEvent(event)


class MainWindow(QMainWindow):
    def __init__(self, args):
        super().__init__()
        signal.signal(signal.SIGINT, self.handle_signal)
        self.alive = True
        if args.host_ip is None:
            self.server = Server(host=True)
            self.is_host = True
        else:
            self.server = Server(host=False, host_ip=args.host_ip)
            self.is_host = False
        self.server.initialize()
        self.database = Database()
        self.comm = CommunicationHandler()
        self.show_barca = False
        self.state_refs_np = None
        self.attributes_np = None
        self.destinations = []
        self.visited = set()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.recording_path = os.path.join(current_dir, 'frames')
        os.makedirs(self.recording_path, exist_ok=True)

        self.setWindowTitle("BFMC DASHBOARD")

        self.setStyleSheet("""
            background-color: black;
        """)

        self.load_nerd_font()

        self.map_widget = MapWidget(self)
        self.cam_widget = CameraWidget(self)
        self.car_widget = CarWidget(self)
        self.barca_widget = BarcaWidget(self)
        self.terminal_widget = TerminalWidget(self)
        self.cam_buttons_widget = ButtonsWidget(self)
        self.sidebar_widget = SidebarWidget(self)

        self.cbs = threading.Thread(target=self.set_callbacks, daemon=True)
        self.cbs.start()

        self.comm.camera_frame_signal.connect(self.cam_widget.process_camera_frame)
        self.comm.depth_frame_signal.connect(self.cam_widget.process_depth_frame)
        self.comm.depth_arr_signal.connect(self.cam_widget.hud.set_depth_arr)
        self.comm.lane_signal.connect(self.cam_widget.lane_callback)
        self.comm.road_obj_signal.connect(self.map_widget.road_objects_callback)
        self.comm.waypoint_signal.connect(self.map_widget.waypoint_callback)
        self.comm.sign_signal.connect(self.handle_sign_update)
        self.comm.steer_signal.connect(self.car_widget.set_steer)
        self.comm.sw_load_signal.connect(self.car_widget.update_sw_load)

        self.comm.start_signal.connect(self.cam_buttons_widget.on_start)
        self.comm.message_signal.connect(self.terminal_widget.add_message)
        self.comm.waypoints_signal.connect(self.map_widget.on_waypoint)
        self.comm.params_signal.connect(self.map_widget.on_params)
        self.comm.run_signal.connect(self.map_widget.call_waypoint_service)
        self.comm.goto_signal.connect(self.cam_buttons_widget.on_goto)
        self.comm.set_states_signal.connect(self.sidebar_widget.on_set_states)

        root_widget = QWidget()
        self.setCentralWidget(root_widget)
        root_layout = QHBoxLayout(root_widget)
        root_layout.setContentsMargins(10, 10, 10, 10)

        self.buttons_overlay = ButtonsOverlay(self)
        self.buttons_overlay.move(80, 5)

        left_widgets = QWidget()
        self.left_layout = QVBoxLayout(left_widgets)
        self.left_layout.setContentsMargins(0, 0, 0, 0)

        top_widgets = QWidget()
        self.top_layout = QHBoxLayout(top_widgets)
        self.top_layout.setContentsMargins(0, 0, 0, 0)
        self.top_layout.addWidget(self.sidebar_widget)
        self.stacked_widget = MapContainer()
        self.stacked_widget.mouse_enter.connect(self.show_mouse_pos)
        self.stacked_widget.mouse_left.connect(self.hide_mouse_pos)
        self.stacked_widget.addWidget(self.map_widget)
        self.stacked_widget.addWidget(self.barca_widget)
        self.top_layout.addWidget(self.stacked_widget)

        self.left_layout.addWidget(top_widgets, 5)
        self.left_layout.addWidget(self.terminal_widget, 2)

        right_widgets = QWidget()
        self.right_layout = QVBoxLayout(right_widgets)
        self.right_layout.setContentsMargins(0, 0, 0, 0)

        self.right_layout.addWidget(self.cam_widget)
        self.right_layout.addWidget(self.cam_buttons_widget)
        self.right_layout.addWidget(self.car_widget)

        root_layout.addWidget(left_widgets, 2)
        root_layout.addWidget(right_widgets, 1)

        self.terminal_widget.add_message("BFMC DASHBOARD INITIALIZED")

        self.udp_timer = QTimer(self)
        self.udp_timer.timeout.connect(self.udp_callbacks)
        self.udp_timer.start(int(CameraParams.FPS_30.value * 1000))

        self.cam_timer = QTimer(self)
        self.cam_timer.timeout.connect(self.cam_record_callback)
        self.cam_timer.start(int(CameraParams.RECORDING_REFRESH_RATE.value * 1000))

        self.cam_thread = threading.Thread(target=self.cam_record_callback, args=(), daemon=True)
        self.cam_thread.start()

    def set_callbacks(self) -> None:
        print("Waiting for TCP client")
        while (self.server.tcp_client is None):
            time.sleep(0.2)
            continue
        print("TCP client connected!")
        self.server.tcp_client.on_start = self.comm.start_signal.emit
        self.server.tcp_client.on_message = self.comm.message_signal.emit
        self.server.tcp_client.on_run = self.comm.run_signal.emit
        self.server.tcp_client.on_goto = self.comm.goto_signal.emit
        self.server.tcp_client.on_set_states = self.comm.set_states_signal.emit
        self.server.tcp_client.on_params = self.comm.params_signal.emit
        self.server.tcp_client.on_waypoint = self.comm.waypoints_signal.emit
        self.server.tcp_client.refresh_run()

    def toggle_map(self) -> None:
        self.show_barca = not self.show_barca
        if self.show_barca:
            self.stacked_widget.setCurrentIndex(1)
        else:
            self.stacked_widget.setCurrentIndex(0)

    def hide_mouse_pos(self) -> None:
        if self.show_barca:
            self.barca_widget.show_mouse = False
        else:
            self.map_widget.show_mouse = False

    def show_mouse_pos(self) -> None:
        if self.show_barca:
            self.barca_widget.show_mouse = True
        else:
            self.map_widget.show_mouse = True

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

    def handle_sign_update(self, sign):
        self.map_widget.sign_callback(sign)
        self.cam_widget.sign_callback(sign)

    def set_destinations(self, destinations):
        self.destinations = destinations

    def reset_run_statistics(self):
        self.visited.clear()
        self.car_widget.run_statistics.dist_traveled = 0
        self.car_widget.run_statistics.set_distance_traveled()
        self.car_widget.run_statistics.set_dest_visited_num(0)
        self.car_widget.run_statistics.set_total_path_distance()
        self.map_widget.update_waypoints()
        self.map_widget.next_destination = None
        self.map_widget.no_destinations = False

    def udp_callbacks(self) -> None:
        rgb_image = None
        depth_image = None
        depth_arr = None
        if self.cam_widget.show_depth:
            depth_image = self.server.udp_connection.parse_depth_image()
        else:
            rgb_image = self.server.udp_connection.parse_rgb_image()

        depth_arr = self.server.udp_connection.parse_depth_arr()
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
        if depth_arr is not None:
            self.comm.depth_arr_signal.emit(depth_arr)
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

    def cam_record_callback(self) -> None:
        if self.cam_buttons_widget.recording:
            rgb_image = self.server.udp_connection.parse_rgb_image()
            if rgb_image is not None:
                now = time.time()
                # if abs(self.car_widget.speed) > 0.02:
                filename = self.recording_path + f"/frame_{int(now)}.jpg"
                rgb_image.save(filename, 'JPG', quality=100)

    def closeEvent(self, event):
        try:
            self.alive = False
            self.car_widget.cleanup_gl_resources()
            self.terminal_widget.terminate_processes()
            time.sleep(0.2)
            print("Processes terminated")
            self.server.udp_connection.alive = False
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


def parse_args():
    p = argparse.ArgumentParser(description="BFMC Dashboard")
    p.add_argument(
        "-ip", "--host-ip",
        nargs="?",          # allow 0 or 1 arguments
        const="127.0.0.1",  # value when "-ip" is given with no following string
        default=None,       # value when "-ip" is omitted entirely
        help="the host IP address (if no IP is provided, use 127.0.0.1)"
    )
    return p.parse_args()


if __name__ == '__main__':
    QApplication.setAttribute(Qt.AA_EnableHighDpiScaling, True)
    QApplication.setAttribute(Qt.AA_UseHighDpiPixmaps, True)

    args = parse_args()
    app = QApplication(sys.argv)

    window = MainWindow(args)
    window.show()

    app.exec()
