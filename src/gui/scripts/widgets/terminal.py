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
        self.kill_thread = None
        self.terminals = deque()
        self.terminals.append(TerminalType.ROS)
        self.message_history = deque([], 1000)
        self.sim_display = None
        self.sim_process = None
        self.sim_widget = SimulatorWidget()
        self.setup_ui()
        self.connect_signals()

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)

        buttons = QWidget()
        self.button_wrapper = QtWidgets.QHBoxLayout(buttons)
        self.buttons = deque()
        self.stop_btn = QtWidgets.QPushButton(' Stop')
        self.ros_btn = QtWidgets.QPushButton(' ROS Debug')
        self.sim_btn = QtWidgets.QPushButton('󰘨 Simulator')
        self.buttons.append(self.stop_btn)
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
        self.stop_btn.setStyleSheet("""
            QPushButton {
                background-color: #ff0000;
            }
            QPushButton:hover {
                background-color: #ff4d4d;
            }
        """)

    def connect_signals(self):
        self.stop_btn.clicked.connect(self.handle_stop_btn_click)
        self.ros_btn.clicked.connect(self.handle_ros_btn_click)
        self.sim_btn.clicked.connect(self.handle_sim_btn_click)

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
        current_terminal_type = self.get_current_terminal_type()
        self.update_button_style(self.ros_btn, current_terminal_type == TerminalType.ROS)
        self.update_button_style(self.sim_btn, current_terminal_type == TerminalType.SIM)

    def find_widget_index(self, term_type):
        if term_type not in self.terminals:
            self.terminals.append(term_type)
            widget_index = len(self.terminals) - 1
        else:
            widget_index = self.terminals.index(term_type)
        return widget_index

    def set_terminal(self, term_type):
        widget_index = self.find_widget_index(term_type)
        self.stacked_widget.setCurrentIndex(widget_index)
        self.update_buttons_style()

    def get_current_terminal_type(self):
        widget_index = self.stacked_widget.currentIndex()
        return self.terminals[widget_index]

    def kill_process(self, process, button, display, terminal_type):
        process.terminate()
        process.waitForFinished()
        self.stacked_widget.removeWidget(display)
        display.deleteLater()
        self.terminals.remove(terminal_type)
        self.stacked_widget.setCurrentIndex(0)
        self.update_buttons_style()

    ################
    # Stop Button
    ################

    def handle_stop_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.SIM:
            self.stop_sim_process()

    ################
    # Simulator
    ################

    def handle_sim_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.SIM:
            return
        if self.sim_display is None:
            self.sim_widget.exec()
            cmd = self.sim_widget.get_cmd()
            if cmd is None:
                return
            self.create_sim_display()
            self.set_terminal(TerminalType.SIM)
            self.start_sim_process(cmd)
        else:
            self.set_terminal(TerminalType.SIM)

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

    def stop_sim_process(self):
        if hasattr(self, 'sim_process') and self.sim_process:
            try:
                self.sim_process.readyReadStandardOutput.disconnect()
            except Exception as e:
                print(e)
            self.kill_process(self.sim_process, self.sim_btn, self.sim_display, TerminalType.SIM)
            self.sim_process = None
            self.sim_display = None

    def read_sim_output(self):
        data = self.sim_process.readAllStandardOutput().data().decode()
        self.sim_display.append(data.strip())

    ################
    # ROS Debug
    ################

    def handle_ros_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.ROS:
            return
        self.stacked_widget.setCurrentIndex(0)
        self.update_buttons_style()

    def add_message(self, message) -> None:
        self.message_history.append(message)
        timestamp = QtCore.QDateTime.currentDateTime().toString("[hh:mm:ss] ")
        self.message_display.setPlainText("\n".join([f"{timestamp}{msg}" for msg in self.message_history]))
        self.message_display.verticalScrollBar().setValue(self.message_display.verticalScrollBar().maximum())
