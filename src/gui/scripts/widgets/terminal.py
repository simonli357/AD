from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QWidget
from collections import deque
from enum import Enum
from .simulator import SimulatorWidget


class TerminalType(Enum):
    ROS = 1
    SIM = 2


class TerminalWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.current_terminal_type = TerminalType.ROS
        self.message_history = deque([], 1000)
        self.sim_display = None
        self.sim_process = None
        self.sim_widget = SimulatorWidget()
        self.setup_ui()
        self.connect_signals()

    def update_button_style(self, button, is_active):
        """Update button color based on boolean state"""
        color = "#ffa500" if is_active else "rgba(255, 255, 255, 0.08);"
        button.setStyleSheet(f"""
            QPushButton {{
                background-color: {color};
            }}
            QPushButton:hover {{
                background-color: #9933ff;
            }}
        """)

    def update_buttons_style(self):
        self.update_button_style(self.ros_btn, self.current_terminal_type == TerminalType.ROS)
        self.update_button_style(self.sim_btn, self.current_terminal_type == TerminalType.SIM)

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)

        buttons = QWidget()
        self.button_wrapper = QtWidgets.QHBoxLayout(buttons)
        self.buttons = deque()
        self.ros_btn = QtWidgets.QPushButton(' ROS Debug')
        self.sim_btn = QtWidgets.QPushButton('󰘨 Simulator')
        self.buttons.append(self.ros_btn)
        self.buttons.append(self.sim_btn)
        for btn in self.buttons:
            self.button_wrapper.addWidget(btn)
        self.button_wrapper.addStretch()

        self.message_display = QtWidgets.QTextEdit()
        self.message_display.setReadOnly(True)

        buttons.setStyleSheet("""
            QPushButton {
                border: none;
                background-color: rgba(255, 255, 255, 0.08);
                padding: 5px 40px 5px 40px;
                color: white;
                font-size: 20px;
            }
        """)

        self.stacked_widget = QtWidgets.QStackedWidget()
        self.stacked_widget.addWidget(self.message_display)

        self.message_display.setStyleSheet("""
            QTextEdit {
                background-color: rgba(255, 255, 255, 0.08);
                font-family: 'Roboto';
                font-size: 20px;
                color: white;
                margin-left: 8px;
                border-radius: 12px;
                padding: 5px;
            }
        """)

        self.layout.addWidget(buttons)
        self.layout.addWidget(self.stacked_widget)

        self.update_buttons_style()

    def connect_signals(self):
        self.ros_btn.clicked.connect(self.handle_ros_btn_click)
        self.sim_btn.clicked.connect(self.handle_sim_btn_click)

    ################
    # Simulator
    ################

    def handle_sim_btn_click(self):
        if self.current_terminal_type == TerminalType.SIM:
            return
        if self.sim_display is None:
            self.sim_widget.exec()
            cmd = self.sim_widget.get_cmd()
            if cmd is None:
                return
            self.current_terminal_type = TerminalType.SIM
            self.update_buttons_style()
            self.create_sim_display()
            self.stacked_widget.setCurrentIndex(1)
            self.start_sim_process(cmd)
        else:
            self.current_terminal_type = TerminalType.SIM
            self.stacked_widget.setCurrentIndex(1)
            self.update_buttons_style()

    def create_sim_display(self):
        self.sim_display = QtWidgets.QTextEdit()
        self.sim_display.setReadOnly(True)
        self.sim_display.setStyleSheet("""
            QTextEdit {
                background-color: rgba(255, 255, 255, 0.08);
                font-family: 'Roboto';
                font-size: 20px;
                color: white;
                margin-left: 8px;
                border-radius: 12px;
                padding: 5px;
            }
        """)
        self.stacked_widget.addWidget(self.sim_display)

    def start_sim_process(self, cmd):
        self.sim_process = QProcess(self)
        self.sim_process.readyReadStandardOutput.connect(self.read_sim_output)
        self.sim_process.start('bash', ['-i', '-c', cmd])

    def read_sim_output(self):
        data = self.sim_process.readAllStandardOutput().data().decode()
        self.sim_display.append(data.strip())

    ################
    # ROS Debug
    ################

    def handle_ros_btn_click(self):
        if self.current_terminal_type == TerminalType.ROS:
            return
        self.current_terminal_type = TerminalType.ROS
        self.stacked_widget.setCurrentIndex(0)
        self.update_buttons_style()

    def add_message(self, message) -> None:
        self.message_history.append(message)
        timestamp = QtCore.QDateTime.currentDateTime().toString("[hh:mm:ss] ")
        self.message_display.setPlainText("\n".join([f"{timestamp}{msg}" for msg in self.message_history]))
        self.message_display.verticalScrollBar().setValue(self.message_display.verticalScrollBar().maximum())
