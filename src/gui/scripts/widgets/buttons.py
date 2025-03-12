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
        self.timer_label = QLabel(' 00:00:<font size="1">00</font>')
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
        self.timer_label.setText(f' {minutes:02d}:{seconds:02d}:<font size="1">{centiseconds:02d}</font>')

    def setup_ui(self) -> None:
        self.window().setAttribute(QtCore.Qt.WA_AlwaysShowToolTips, True)
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.setContentsMargins(0, 5, 0, 0)
        self.layout.setSpacing(10)
        self.layout.setAlignment(QtCore.Qt.AlignLeft)
        self.buttons = deque()

        self.start_btn = QtWidgets.QPushButton("")
        self.stop_btn = QtWidgets.QPushButton("")
        self.goto_btn = QtWidgets.QPushButton("󰓾")

        self.buttons.append(self.start_btn)
        self.buttons.append(self.stop_btn)
        self.buttons.append(self.goto_btn)

        self.start_btn.setToolTip("Start/Pause/Resume")
        self.stop_btn.setToolTip("Stop")
        self.goto_btn.setToolTip("Go To")

        for btn in self.buttons:
            self.layout.addWidget(btn, 1)

        self.layout.addWidget(self.timer_label, 2)

        self.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 255, 255, 0.08);
                padding: 12px 36px 12px 32px;
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
            QLabel {
                background-color: rgba(255, 255, 255, 0.08);
                border-radius: 8px;
                color: white;
                font-size: 28px;
            }
        """)

        self.update_button_style(self.start_btn, self.started)

    def update_button_style(self, button, is_active):
        """Update button color based on boolean state"""
        color = "#FF0000" if is_active else "rgba(0, 255, 0, 1.0);"  # Light green/red
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
            }}
            QPushButton:hover {{
                background-color: #9933ff;
            }}
        """)

    def connect_signals(self) -> None:
        self.stop_btn.clicked.connect(self.handle_stop_click)
        self.start_btn.clicked.connect(self.handle_start_click)
        self.goto_btn.clicked.connect(self.handle_goto_click)

    def toggle_start_icon(self) -> None:
        if self.started:
            self.start_btn.setText("")
        else:
            self.start_btn.setText("")
        self.update_button_style(self.start_btn, self.started)

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
            self.started = True
            if self.start_time is None:
                self.start_time = time.time()
            else:
                self.start_time = time.time() - self.accumulated_centiseconds / 100
            self.time_timer.start(25)
        else:
            print("Stopping")
            self.started = False
            if self.start_time is not None:
                elapsed = time.time() - self.start_time
                self.accumulated_centiseconds += int(elapsed * 100)
                self.start_time = None
            self.time_timer.stop()
        self.toggle_start_icon()

    def handle_stop_click(self) -> None:
        self.time_timer.stop()
        self.start_time = None
        self.timer_label.setText(' 00:00:<font size="1">00</font>')

    def handle_goto_click(self) -> None:
        print("Planning Path")
        if self.main_window.map_widget.cursor_coords:
            self.call_goto_service(self.main_window.map_widget.cursor_coords)
        else:
            print("Not a valid destination")
