from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QTimer
from PyQt5.QtWidgets import QLabel
from collections import deque

import time
import numpy as np


class ButtonsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.server = self.main_window.server
        self.started = False
        self.start_time = None
        self.accumulated_centiseconds = 0
        self.timer_label = QLabel('00:00:00')
        self.timer_label.setAlignment(QtCore.Qt.AlignCenter)
        self.time_timer = QTimer(self)
        self.time_timer.timeout.connect(self.update_stopwatch)
        self.setup_ui()
        self.connect_signals()

    def update_stopwatch(self) -> None:
        if self.start_time is None:
            return
        current_time = time.time()
        elapsed_seconds = current_time - self.start_time
        total_centiseconds = int(elapsed_seconds * 100) + self.accumulated_centiseconds
        minutes = (total_centiseconds // 6000) % 60
        seconds = (total_centiseconds // 100) % 60
        centiseconds = total_centiseconds % 100
        # self.centiseconds += 1
        # minutes = (self.centiseconds // 6000) % 60
        # seconds = (self.centiseconds // 100) % 60
        # centiseconds = self.centiseconds % 100
        self.timer_label.setText(f'{minutes:02d}:{seconds:02d}:{centiseconds:02d}')

    def setup_ui(self) -> None:
        self.window().setAttribute(QtCore.Qt.WA_AlwaysShowToolTips, True)
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.buttons = deque()

        self.start_btn = QtWidgets.QPushButton("")
        self.goto_btn = QtWidgets.QPushButton("󰓾")

        self.buttons.append(self.start_btn)
        self.buttons.append(self.goto_btn)

        self.start_btn.setToolTip("Start/Stop")
        self.goto_btn.setToolTip("Go To")

        for btn in self.buttons:
            self.layout.addWidget(btn)

        self.layout.addWidget(self.timer_label)

        self.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 255, 255, 0.15);
                padding: 12px 36px 12px 32px;
                margin-right: 12px;
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
            QToolTip {
                background-color: black;
                color: white;
                border: none;
                font-size: 14px;
                padding: 5px;
            }
        """)

    def connect_signals(self) -> None:
        self.start_btn.clicked.connect(self.handle_start_click)
        self.goto_btn.clicked.connect(self.handle_goto_click)

    def toggle_start_icon(self) -> None:
        if self.start_btn.text() == "":
            self.start_btn.setText("")
        else:
            self.start_btn.setText("")

    def call_start_service(self, start) -> None:
        try:
            if self.server.utility_node_client.socket is None:
                return
            self.server.utility_node_client.send_start_srv(not self.started)
            max_retries = 50
            retries = 0
            while (retries < max_retries):
                if self.server.utility_node_client.start_srv_msg:
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to start/stop")
        except Exception as e:
            print(e)

    def call_goto_service(self, cursor_coords):
        try:
            if self.server.utility_node_client.socket is None:
                return
            if len(cursor_coords) == 0:
                self.server.utility_node_client.send_go_to_cmd_srv([(self.main_window.map_widget.cursor_x, self.main_window.map_widget.cursor_y)])
            else:
                self.server.utility_node_client.send_go_to_cmd_srv(cursor_coords)
            max_retries = 50
            retries = 0
            res = self.server.utility_node_client.go_to_cmd_srv_msg
            while (retries < max_retries):
                if (len(res.state_refs.data) > 0 and len(res.wp_attributes.data) > 0):
                    self.main_window.map_widget.state_refs_np = np.array(res.state_refs.data).reshape(3, -1)
                    self.main_window.map_widget.attributes_np = np.array(res.wp_attributes.data)
                    print("Goto_command service call successful. shape: ", self.main_window.map_widget.state_refs_np.shape)
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to send go to cmd")
        except Exception as e:
            print(e)

    def handle_start_click(self) -> None:
        self.call_start_service(not self.started)
        if not self.started:
            print("Starting")
            if self.start_time is None:
                self.start_time = time.time()
            else:
                self.start_time = time.time() - self.accumulated_centiseconds / 100
            self.time_timer.start(25)
        else:
            print("Stopping")
        self.toggle_start_icon()

    def handle_goto_click(self) -> None:
        print("Planning Path")
        self.call_goto_service(self.main_window.map_widget.cursor_coords)
