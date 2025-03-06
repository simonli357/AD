from PyQt5.QtWidgets import QVBoxLayout, QLineEdit, QDialog, QDialogButtonBox
from PyQt5 import QtCore

import os
import yaml


class SimulatorWidget(QDialog):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(QtCore.QSize(800, 150))
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(current_dir, 'appdata')
        self.config = os.path.join(data_dir, 'simulator.yaml')

        if not os.path.exists(data_dir):
            os.makedirs(data_dir)

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

        # UI setup
        layout = QVBoxLayout()
        self.catkin_ws = QLineEdit(self)
        self.catkin_ws.setPlaceholderText(self.catkin_ws_cached)

        self.args = QLineEdit(self)
        self.args.setPlaceholderText(self.args_cached)

        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        layout.addWidget(self.catkin_ws)
        layout.addWidget(self.args)
        layout.addWidget(self.buttonBox)

        self.setLayout(layout)

    def create_default_config(self):
        default_config = {
            'catkin_ws': '/path/to/catkin_ws',
            'args': 'run3.launch'
        }
        os.makedirs(os.path.dirname(self.config), exist_ok=True)
        with open(self.config, 'w') as file:
            yaml.dump(default_config, file)

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
        self.cmd = f'{src_ros} && cd {catkin_ws} && {src_devel} && roslaunch sim_pkg {args}'
        with open(self.config, 'w') as file:
            yaml.dump(self.cache, file)
        self.catkin_ws.clear()
        self.args.clear()
        super().accept()

    def reject(self):
        self.catkin_ws.clear()
        self.args.clear()
        super().reject()

    def get_cmd(self):
        cmd = self.cmd
        self.cmd = None
        return cmd
