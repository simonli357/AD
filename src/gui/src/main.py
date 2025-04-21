#!/usr/bin/env python3

import sys
import os
import time
import threading
import signal
import cv2

from PyQt5.QtWidgets import QApplication, QMainWindow, QHBoxLayout, QVBoxLayout, QWidget
from PyQt5.QtGui import QFontDatabase, QFont
from PyQt5 import QtWidgets, QtGui
from PyQt5.QtCore import pyqtSignal, QObject, Qt
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
from std_srvs.srv import TriggerRequest


class CommunicationHandler(QObject):
    message_signal = pyqtSignal(str)
    params_signal = pyqtSignal(object, object)
    camera_frame_signal = pyqtSignal(QtGui.QPixmap)
    depth_frame_signal = pyqtSignal(QtGui.QPixmap)
    lane_signal = pyqtSignal(object)
    road_obj_signal = pyqtSignal(object)
    waypoint_signal = pyqtSignal(object)
    sign_signal = pyqtSignal(object)
    run_signal = pyqtSignal(object)
    steer_signal = pyqtSignal(object)
    render_widget_signal = pyqtSignal()
    render_barca_widget_signal = pyqtSignal()
    render_map_widget_signal = pyqtSignal()
    sw_load_signal = pyqtSignal(object)
    graph_signal = pyqtSignal(object)


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
    def __init__(self, server):
        super().__init__()
        signal.signal(signal.SIGINT, self.handle_signal)
        self.alive = True
        self.server = server
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

        self.comm.message_signal.connect(self.terminal_widget.add_message)
        self.comm.params_signal.connect(self.handle_params_update)
        self.comm.camera_frame_signal.connect(self.cam_widget.process_camera_frame)
        self.comm.depth_frame_signal.connect(self.cam_widget.process_depth_frame)
        self.comm.lane_signal.connect(self.cam_widget.lane_callback)
        self.comm.road_obj_signal.connect(self.map_widget.road_objects_callback)
        self.comm.waypoint_signal.connect(self.map_widget.waypoint_callback)
        self.comm.sign_signal.connect(self.handle_sign_update)
        self.comm.run_signal.connect(self.map_widget.call_waypoint_service)
        self.comm.steer_signal.connect(self.car_widget.set_steer)
        self.comm.sw_load_signal.connect(self.car_widget.update_sw_load)
        self.comm.graph_signal.connect(self.map_widget.update_graph)

        self.comm.render_widget_signal.connect(self.car_widget.render_widget)
        self.comm.render_widget_signal.connect(self.cam_widget.update_hud)

        self.comm.render_barca_widget_signal.connect(self.barca_widget.render_widget)
        self.comm.render_map_widget_signal.connect(self.map_widget.render_widget)

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

        self.terminal_widget.add_message("AD IDE INITIALIZED")

        self.udp_thread = threading.Thread(target=self.udp_callbacks, args=(), daemon=True)
        self.tcp_thread = threading.Thread(target=self.tcp_callbacks, args=(), daemon=True)
        self.cam_thread = threading.Thread(target=self.cam_record_callback, args=(), daemon=True)
        self.udp_thread.start()
        self.tcp_thread.start()
        self.cam_thread.start()

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

    def handle_params_update(self, req, res):
        response = self.map_widget.update_params(req)
        self.server.utility_node_client.send_trigger(TriggerRequest(), response)

    def handle_sign_update(self, sign):
        self.map_widget.sign_callback(sign)
        self.cam_widget.sign_callback(sign)

    def set_destinations(self, destinations):
        self.destinations = destinations

    def reset_run_statistics(self):
        self.car_widget.run_statistics.dist_traveled = 0
        self.car_widget.run_statistics.set_distance_traveled()
        self.car_widget.run_statistics.visited.clear()
        self.car_widget.run_statistics.set_dest_visited_num(0)
        self.car_widget.run_statistics.set_total_path_distance()
        self.map_widget.update_waypoints()
        self.map_widget.next_destination = None
        self.map_widget.no_destinations = False

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
                if self.server.utility_node_client.graph_msg:
                    graph = self.server.utility_node_client.graph_msg.popleft()
                    self.comm.graph_signal.emit(graph)
            self.render_callbacks()
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
            if self.cam_buttons_widget.recording:
                rgb_image = self.server.udp_connection.parse_rgb_image()
                if rgb_image is not None:
                    now = time.time()
                    if self.recording and abs(self.meter_widget.speed) > 0.02:
                        filename = self.recording_path + f"/frame_{int(now)}.jpg"
                        cv2.imwrite(filename, rgb_image)
                time.sleep(CameraParams.RECORDING_REFRESH_RATE.value)
            else:
                time.sleep(CameraParams.RECORDING_REFRESH_RATE.value)

    def render_callbacks(self) -> None:
        self.comm.render_widget_signal.emit()
        if self.show_barca:
            self.comm.render_barca_widget_signal.emit()
        else:
            self.comm.render_map_widget_signal.emit()

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
