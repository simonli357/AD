from PyQt5.QtWidgets import QWidget, QVBoxLayout, QHBoxLayout, QLineEdit, QDialog, QDialogButtonBox, QLabel
from PyQt5 import QtCore, QtWidgets

import os
import yaml


class SimulatorFormWidget(QDialog):
    def __init__(self):
        super().__init__()
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMinimumWidth(600)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(current_dir, 'appdata')
        self.config = os.path.join(self.data_dir, 'simulator.yaml')
        self.load_cache()
        self.setup_ui()
        self.setStyleSheet("""
            background-color: rgba(255, 255, 255, 0.05);
            color: white;
            font-size: 16px;
        """)

    def setup_ui(self):
        self.catkin_ws = QLineEdit(self)
        self.catkin_ws.setText(self.catkin_ws_cached)
        self.catkin_ws_label = QLabel('Path  ')
        self.catkin_ws_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.args = QLineEdit(self)
        self.args.setText(self.args_cached)
        self.args_label = QLabel('Args 󰦨 ')
        self.args_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        title = QLabel("󰘨 Simulator")
        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 32px;
                color: orange;
            }
        """)

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        # Get the OK and Cancel buttons
        ok_button = self.buttonBox.button(QDialogButtonBox.Ok)
        cancel_button = self.buttonBox.button(QDialogButtonBox.Cancel)

        # Assign unique object names for styling
        ok_button.setObjectName("okButton")
        cancel_button.setObjectName("cancelButton")

        # Apply styles
        self.buttonBox.setStyleSheet("""
            QPushButton#okButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px 16px;
            }
            QPushButton#cancelButton {
                background-color: #f44336;
                color: white;
                border: none;
                padding: 8px 16px;
            }
            QPushButton:hover {
                opacity: 0.9;
            }
        """)

        layout = QVBoxLayout()
        layout.setAlignment(QtCore.Qt.AlignCenter)
        layout.addStretch()

        layout.addWidget(title)

        labels = QWidget()
        labels_layout = QVBoxLayout(labels)
        labels_layout.setAlignment(QtCore.Qt.AlignLeft)
        labels_layout.addWidget(self.catkin_ws_label)
        labels_layout.addWidget(self.args_label)

        text_fields = QWidget()
        text_fields_layout = QVBoxLayout(text_fields)
        text_fields_layout.setAlignment(QtCore.Qt.AlignLeft)
        text_fields_layout.addWidget(self.catkin_ws)
        text_fields_layout.addWidget(self.args)

        form = QWidget()
        form.setStyleSheet("""
            QLineEdit {
                background-color: transparent;
                padding: 5px 5px 5px 5px;
                border: 1px solid rgba(255,255,255,0.25);
                border-radius: 4px;
            }
            QLabel {
                background-color: transparent;
                padding: 5px 5px 5px 5px;
                color: #0099ff
            }
        """)

        form_layout = QHBoxLayout(form)
        form_layout.addWidget(labels)
        form_layout.addWidget(text_fields)

        layout.addWidget(form)
        layout.addWidget(self.buttonBox)

        self.setLayout(layout)

    def load_cache(self):
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)

        if not os.path.isfile(self.config):
            self.create_default_config()

        self.catkin_ws_cached = None
        self.args_cached = None
        self.cache = None
        self.cmd = None

        with open(self.config, 'r') as file:
            self.cache = yaml.safe_load(file)
            self.catkin_ws_cached = self.cache['catkin_ws']
            self.args_cached = self.cache['args']

    def create_default_config(self):
        default_config = {
            'catkin_ws': '/path/to/catkin_ws',
            'args': 'run3.launch'
        }
        os.makedirs(os.path.dirname(self.config), exist_ok=True)
        with open(self.config, 'w') as file:
            yaml.dump(default_config, file)

    def clear_inputs(self):
        self.catkin_ws.clear()
        self.args.clear()

    def accept(self):
        src_ros = 'source /opt/ros/noetic/setup.sh'
        src_devel = 'source devel/setup.bash'
        if self.args_cached == 'args':
            self.args_cached = ''
        catkin_ws = self.catkin_ws.text()
        args = self.args.text()
        if not catkin_ws:
            catkin_ws = self.catkin_ws_cached
        else:
            self.cache['catkin_ws'] = catkin_ws
        if not args:
            args = self.args_cached
        else:
            self.cache['args'] = args
        bash = 'exec bash -c'
        command = f'{src_ros} && cd {catkin_ws} && {src_devel} && roslaunch sim_pkg {args}'
        self.cmd = f'{bash} "{command}"'
        with open(self.config, 'w') as file:
            yaml.dump(self.cache, file)
        self.clear_inputs()
        super().accept()

    def reject(self):
        self.clear_inputs()
        super().reject()

    def get_cmd(self):
        cmd = self.cmd
        self.cmd = None
        return cmd
