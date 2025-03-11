from PyQt5 import QtWidgets, QtCore
from PyQt5.QtCore import QProcess
from PyQt5.QtWidgets import QWidget
from collections import deque
from .enums import TerminalType
from .forms.simulator_form import SimulatorFormWidget
from .forms.compile_form import CompileFormWidget
from .forms.ssh_form import SSHFormWidget
from .indicators.progress_window import ProgressWindow

import re


class TerminalWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.kill_thread = None
        self.terminals = deque()
        self.terminals.append(TerminalType.DEBUG)
        # ROS debug
        self.message_history = deque([], 1000)
        # Compile
        self.compile_process = None
        # Simulator
        self.sim_display = None
        self.sim_process = None
        # Control
        self.ctrl_display = None
        self.ctrl_process = None
        # Camera
        self.cam_display = None
        self.cam_process = None
        # Path planner
        self.path_display = None
        self.path_process = None
        # Roscore
        self.roscore_display = None
        self.roscore_process = None
        self.setup_ui()
        self.connect_signals()

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)

        buttons = QWidget()
        self.button_wrapper = QtWidgets.QHBoxLayout(buttons)
        self.buttons = deque()
        self.stop_btn = QtWidgets.QPushButton(' stop')
        self.compile_btn = QtWidgets.QPushButton(' build')
        self.debug_btn = QtWidgets.QPushButton(' debug')
        self.sim_btn = QtWidgets.QPushButton('󰘨 simulator')
        self.controller_btn = QtWidgets.QPushButton('󱡸 controller')
        self.cam_btn = QtWidgets.QPushButton('  camera')
        self.path_planner_btn = QtWidgets.QPushButton('  planner')
        self.roscore_btn = QtWidgets.QPushButton(' roscore')
        self.buttons.append(self.stop_btn)
        self.buttons.append(self.compile_btn)
        self.buttons.append(self.debug_btn)
        self.buttons.append(self.sim_btn)
        self.buttons.append(self.controller_btn)
        self.buttons.append(self.cam_btn)
        self.buttons.append(self.path_planner_btn)
        self.buttons.append(self.roscore_btn)
        for btn in self.buttons:
            self.button_wrapper.addWidget(btn)
        self.button_wrapper.addStretch()

        self.message_display = QtWidgets.QTextEdit()
        self.message_display.setReadOnly(True)

        buttons.setStyleSheet("""
            QPushButton {
                border: none;
                background-color: rgba(255, 255, 255, 0.08);
                padding: 5px 20px 5px 20px;
                color: white;
                font-size: 18px;
                border-radius: 8px;
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
                padding: 5px 15px 5px 15px;
            }
            QPushButton:hover {
                background-color: #ff4d4d;
            }
        """)
        self.compile_btn.setStyleSheet("""
            QPushButton {
                background-color: #008000;
                padding: 5px 15px 5px 15px;
            }
            QPushButton:hover {
                background-color: rgba(0,144,0,0.25);
            }
        """)

    def connect_signals(self):
        self.stop_btn.clicked.connect(self.handle_stop_btn_click)
        self.compile_btn.clicked.connect(self.handle_compile_btn_click)
        self.debug_btn.clicked.connect(self.handle_ros_btn_click)
        self.sim_btn.clicked.connect(self.handle_sim_btn_click)
        self.controller_btn.clicked.connect(self.handle_controller_btn_click)
        self.cam_btn.clicked.connect(self.handle_cam_btn_click)
        self.path_planner_btn.clicked.connect(self.handle_path_planner_btn_click)
        self.roscore_btn.clicked.connect(self.handle_roscore_btn_click)

    def update_button_style(self, button, is_active):
        """Update button color based on boolean state"""
        if (is_active):
            color = "#ffa500"
            button.setStyleSheet(f"""
                QPushButton {{
                    background-color: {color};
                }}
                QPushButton:hover {{
                    background-color: #9933ff;
                }}
            """)

    def activate_button(self, button, is_active):
        """Update button color based on boolean state"""
        color = "rgba(255, 255, 0, 0.25)" if is_active else "rgba(255, 255, 255, 0.08);"
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
        self.activate_button(self.debug_btn, TerminalType.DEBUG in self.terminals)
        self.activate_button(self.sim_btn, TerminalType.SIM in self.terminals)
        self.activate_button(self.controller_btn, TerminalType.CONTROL in self.terminals)
        self.activate_button(self.cam_btn, TerminalType.CAM in self.terminals)
        self.activate_button(self.path_planner_btn, TerminalType.PATH in self.terminals)
        self.activate_button(self.roscore_btn, TerminalType.ROSCORE in self.terminals)
        self.update_button_style(self.debug_btn, current_terminal_type == TerminalType.DEBUG)
        self.update_button_style(self.sim_btn, current_terminal_type == TerminalType.SIM)
        self.update_button_style(self.controller_btn, current_terminal_type == TerminalType.CONTROL)
        self.update_button_style(self.cam_btn, current_terminal_type == TerminalType.CAM)
        self.update_button_style(self.path_planner_btn, current_terminal_type == TerminalType.PATH)
        self.update_button_style(self.roscore_btn, current_terminal_type == TerminalType.ROSCORE)

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

    def kill_process(self, process, display, terminal_type):
        process.terminate()
        process.waitForFinished()
        self.stacked_widget.removeWidget(display)
        display.deleteLater()
        self.terminals.remove(terminal_type)
        self.stacked_widget.setCurrentIndex(0)
        self.update_buttons_style()

    ################
    # Stop
    ################

    def handle_stop_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.SIM:
            self.stop_sim_process()
        elif current_terminal_type == TerminalType.CONTROL:
            self.stop_ctrl_process()
        elif current_terminal_type == TerminalType.CAM:
            self.stop_cam_process()
        elif current_terminal_type == TerminalType.PATH:
            self.stop_path_process()
        elif current_terminal_type == TerminalType.ROSCORE:
            self.stop_roscore_process()

    ################
    # Compile
    ################
    def handle_compile_btn_click(self):
        modal = CompileFormWidget()
        modal.exec()
        cmd = modal.get_cmd()
        if cmd is None:
            return
        self.start_compile_process(cmd)

    def read_compile_output(self):
        stdout = self.compile_process.readAllStandardOutput().data().decode()
        if stdout:
            # Process each line to find the latest percentage
            for line in stdout.splitlines():
                line = line.strip()
                match = re.search(r'(\d+)%', line)
                if match:
                    val = int(match.group(1))
                    if self.compile_progress is not None:
                        self.compile_progress.set_progress(val)
                else:
                    self.compile_progress.increment(1)

    def start_compile_process(self, cmd):
        self.compile_process = QProcess(self)
        self.compile_process.readyReadStandardOutput.connect(self.read_compile_output)
        self.compile_process.start('bash', ['-c', cmd])
        self.compile_progress = ProgressWindow()
        self.compile_progress.exec()

    ################
    # Simulator
    ################

    def handle_sim_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.SIM:
            return
        if self.sim_display is None:
            modal = SimulatorFormWidget()
            modal.exec()
            cmd = modal.get_cmd()
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

    def read_sim_output(self):
        stdout = self.sim_process.readAllStandardOutput().data().decode()
        if stdout:
            self.sim_display.append(stdout.strip())

    def read_sim_err_output(self):
        stderr = self.sim_process.readAllStandardOutput().data().decode()
        if stderr:
            self.sim_display.append(stderr.strip())

    def start_sim_process(self, cmd):
        self.sim_process = QProcess(self)
        self.sim_process.readyReadStandardOutput.connect(self.read_sim_output)
        self.sim_process.readyReadStandardError.connect(self.read_sim_err_output)
        self.sim_process.start('bash', ['-c', cmd])

    def stop_sim_process(self):
        if hasattr(self, 'sim_process') and self.sim_process:
            try:
                self.sim_process.readyReadStandardOutput.disconnect()
                self.sim_process.readyReadStandardError.disconnect()
            except Exception as e:
                print(e)
            self.kill_process(self.sim_process, self.sim_display, TerminalType.SIM)
            self.sim_process = None
            self.sim_display = None

    ################
    # Controller
    ################

    def handle_controller_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.CONTROL:
            return
        if self.ctrl_display is None:
            modal = SSHFormWidget(TerminalType.CONTROL)
            modal.exec()
            cmd = modal.get_cmd()
            if cmd is None:
                return
            self.create_ctrl_display()
            self.set_terminal(TerminalType.CONTROL)
            self.start_ctrl_process(cmd)
        else:
            self.set_terminal(TerminalType.CONTROL)

    def create_ctrl_display(self):
        self.ctrl_display = QtWidgets.QTextEdit()
        self.ctrl_display.setReadOnly(True)
        self.ctrl_display.setStyleSheet("""
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
        self.stacked_widget.addWidget(self.ctrl_display)

    def read_ctrl_output(self):
        stdout = self.ctrl_process.readAllStandardOutput().data().decode()
        if stdout:
            self.ctrl_display.append(stdout.strip())

    def read_ctrl_err_output(self):
        stderr = self.ctrl_process.readAllStandardError().data().decode()
        if stderr:
            self.ctrl_display.append(stderr.strip())

    def start_ctrl_process(self, cmd):
        self.ctrl_process = QProcess(self)
        self.ctrl_process.readyReadStandardOutput.connect(self.read_ctrl_output)
        self.ctrl_process.readyReadStandardError.connect(self.read_ctrl_err_output)
        self.ctrl_process.start('bash', ['-c', cmd])

    def stop_ctrl_process(self):
        if hasattr(self, 'ctrl_process') and self.ctrl_process:
            try:
                self.ctrl_process.readyReadStandardOutput.disconnect()
                self.ctrl_process.readyReadStandardError.disconnect()
            except Exception as e:
                print(e)
            self.kill_process(self.ctrl_process, self.ctrl_display, TerminalType.CONTROL)
            self.ctrl_process = None
            self.ctrl_display = None

    ################
    # Camera Node
    ################

    def handle_cam_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.CAM:
            return
        if self.cam_display is None:
            modal = SSHFormWidget(TerminalType.CAM)
            modal.exec()
            cmd = modal.get_cmd()
            if cmd is None:
                return
            self.create_cam_display()
            self.set_terminal(TerminalType.CAM)
            self.start_cam_process(cmd)
        else:
            self.set_terminal(TerminalType.CAM)

    def create_cam_display(self):
        self.cam_display = QtWidgets.QTextEdit()
        self.cam_display.setReadOnly(True)
        self.cam_display.setStyleSheet("""
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
        self.stacked_widget.addWidget(self.cam_display)

    def read_cam_output(self):
        stdout = self.cam_process.readAllStandardOutput().data().decode()
        if stdout:
            self.cam_display.append(stdout.strip())

    def read_cam_err_output(self):
        stderr = self.cam_process.readAllStandardError().data().decode()
        if stderr:
            self.cam_display.append(stderr.strip())

    def start_cam_process(self, cmd):
        self.cam_process = QProcess(self)
        self.cam_process.readyReadStandardOutput.connect(self.read_cam_output)
        self.cam_process.readyReadStandardError.connect(self.read_cam_err_output)
        self.cam_process.start('bash', ['-c', cmd])

    def stop_cam_process(self):
        if hasattr(self, 'cam_process') and self.cam_process:
            try:
                self.cam_process.readyReadStandardOutput.disconnect()
                self.cam_process.readyReadStandardError.disconnect()
            except Exception as e:
                print(e)
            self.kill_process(self.cam_process, self.cam_display, TerminalType.CAM)
            self.cam_process = None
            self.cam_display = None

    ################
    # Path Planner
    ################

    def handle_path_planner_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.PATH:
            return
        if self.path_display is None:
            modal = SSHFormWidget(TerminalType.PATH)
            modal.exec()
            cmd = modal.get_cmd()
            if cmd is None:
                return
            self.create_path_display()
            self.set_terminal(TerminalType.PATH)
            self.start_path_process(cmd)
        else:
            self.set_terminal(TerminalType.PATH)

    def create_path_display(self):
        self.path_display = QtWidgets.QTextEdit()
        self.path_display.setReadOnly(True)
        self.path_display.setStyleSheet("""
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
        self.stacked_widget.addWidget(self.path_display)

    def read_path_output(self):
        stdout = self.path_process.readAllStandardOutput().data().decode()
        if stdout:
            self.path_display.append(stdout.strip())

    def read_path_err_output(self):
        stderr = self.path_process.readAllStandardError().data().decode()
        if stderr:
            self.path_display.append(stderr.strip())

    def start_path_process(self, cmd):
        self.path_process = QProcess(self)
        self.path_process.readyReadStandardOutput.connect(self.read_path_output)
        self.path_process.readyReadStandardError.connect(self.read_path_err_output)
        self.path_process.start('bash', ['-c', cmd])

    def stop_path_process(self):
        if hasattr(self, 'path_process') and self.path_process:
            try:
                self.path_process.readyReadStandardOutput.disconnect()
                self.path_process.readyReadStandardError.disconnect()
            except Exception as e:
                print(e)
            self.kill_process(self.path_process, self.path_display, TerminalType.PATH)
            self.path_process = None
            self.path_display = None

    ################
    # Roscore
    ################

    def handle_roscore_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.ROSCORE:
            return
        if self.roscore_display is None:
            modal = SSHFormWidget(TerminalType.ROSCORE)
            modal.exec()
            cmd = modal.get_cmd()
            if cmd is None:
                return
            self.create_roscore_display()
            self.set_terminal(TerminalType.ROSCORE)
            self.start_roscore_process(cmd)
        else:
            self.set_terminal(TerminalType.ROSCORE)

    def create_roscore_display(self):
        self.roscore_display = QtWidgets.QTextEdit()
        self.roscore_display.setReadOnly(True)
        self.roscore_display.setStyleSheet("""
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
        self.stacked_widget.addWidget(self.roscore_display)

    def read_roscore_output(self):
        stdout = self.roscore_process.readAllStandardOutput().data().decode()
        if stdout:
            self.roscore_display.append(stdout.strip())

    def read_roscore_err_output(self):
        stderr = self.roscore_process.readAllStandardError().data().decode()
        if stderr:
            self.roscore_display.append(stderr.strip())

    def start_roscore_process(self, cmd):
        self.roscore_process = QProcess(self)
        self.roscore_process.readyReadStandardOutput.connect(self.read_roscore_output)
        self.roscore_process.readyReadStandardError.connect(self.read_roscore_err_output)
        self.roscore_process.start('bash', ['-c', cmd])

    def stop_roscore_process(self):
        if hasattr(self, 'roscore_process') and self.roscore_process:
            try:
                self.roscore_process.readyReadStandardOutput.disconnect()
                self.roscore_process.readyReadStandardError.disconnect()
            except Exception as e:
                print(e)
            self.kill_process(self.roscore_process, self.roscore_display, TerminalType.ROSCORE)
            self.roscore_process = None
            self.roscore_display = None

    ################
    # Debug
    ################

    def handle_ros_btn_click(self):
        current_terminal_type = self.get_current_terminal_type()
        if current_terminal_type == TerminalType.DEBUG:
            return
        self.stacked_widget.setCurrentIndex(0)
        self.update_buttons_style()

    def add_message(self, message) -> None:
        self.message_history.append(message)
        timestamp = QtCore.QDateTime.currentDateTime().toString("[hh:mm:ss] ")
        self.message_display.setPlainText("\n".join([f"{timestamp}{msg}" for msg in self.message_history]))
        self.message_display.verticalScrollBar().setValue(self.message_display.verticalScrollBar().maximum())
