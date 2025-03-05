from PyQt5.QtWidgets import QVBoxLayout, QLineEdit, QDialog, QDialogButtonBox
from PyQt5 import QtCore

import os
import yaml


class SimulatorWidget(QDialog):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(QtCore.QSize(800, 150))
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(current_dir, 'data')
        self.config = os.path.join(data_dir, 'simulator.yaml')

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

    def accept(self):
        if not self.catkin_ws.text() or not self.args.text():
            self.cmd = f'cd {self.catkin_ws_cached} && source devel/setup.bash && roslaunch sim_pkg {self.args_cached}'
        else:
            self.cmd = f'cd {self.catkin_ws.text()} && source devel/setup.bash && roslaunch sim_pkg {self.args.text()}'
            self.cache['catkin_ws'] = self.catkin_ws.text()
            self.cache['args'] = self.args.text()
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
