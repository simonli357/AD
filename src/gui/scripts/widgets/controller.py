from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QLineEdit, QDialog, QDialogButtonBox, QWidget, QLabel
from PyQt5 import QtCore
from .animated_toggle import AnimatedToggle

import os
import yaml


class ControllerWidget(QDialog):
    def __init__(self):
        super().__init__()
        self.setMinimumSize(QtCore.QSize(400, 275))
        current_dir = os.path.dirname(os.path.abspath(__file__))
        data_dir = os.path.join(current_dir, 'appdata')
        self.config = os.path.join(data_dir, 'controller.yaml')
        self.ssh_config = os.path.join(data_dir, 'ssh.yaml')

        if not os.path.exists(data_dir):
            os.makedirs(data_dir)
        if not os.path.isfile(self.config):
            self.create_default_config()
        if not os.path.isfile(self.ssh_config):
            self.create_default_ssh_config()

        self.catkin_ws_cached = None
        self.args_cached = None
        self.ssh_target_cached = None
        self.passwd_cached = None
        self.remote_catkin_ws_cached = None
        self.cache = None
        self.ssh_cache = None
        self.cmd = None
        self.use_ssh = False

        with open(self.config, 'r') as file:
            self.cache = yaml.safe_load(file)
            self.catkin_ws_cached = self.cache['catkin_ws']
            self.args_cached = self.cache['args']

        with open(self.ssh_config, 'r') as file:
            self.ssh_cache = yaml.safe_load(file)
            self.ssh_target_cached = self.ssh_cache['ssh_target']
            self.passwd_cached = self.ssh_cache['passwd']
            self.remote_catkin_ws_cached = self.ssh_cache['remote_catkin_ws']

        # UI setup
        self.ssh = AnimatedToggle()
        self.ssh.stateChanged.connect(self.handle_ssh_toggle)

        self.catkin_ws = QLineEdit(self)
        self.catkin_ws.setPlaceholderText(self.catkin_ws_cached)

        self.args = QLineEdit(self)
        self.args.setPlaceholderText(self.args_cached)

        self.ssh_target = QLineEdit(self)
        self.ssh_target.setPlaceholderText(self.ssh_target_cached)

        self.passwd = QLineEdit(self)
        self.passwd.setPlaceholderText(self.passwd_cached)

        self.remote_catkin_ws = QLineEdit(self)
        self.remote_catkin_ws.setPlaceholderText(self.remote_catkin_ws_cached)

        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)

        self.layout = QVBoxLayout()
        self.layout.addStretch()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)

        title = QLabel("󱡸 Controller Node")
        title.setStyleSheet("""
            QLabel {
                font-size: 16px;
                margin-left: 6px;
            }
        """)
        self.layout.addWidget(title)

        ssh_opt = QLabel(" Use SSH")
        ssh_opt.setStyleSheet("""
            QLabel {
                font-size: 16px;
            }
        """)

        wrapper = QWidget()
        wrapper_layout = QHBoxLayout(wrapper)
        wrapper_layout.setAlignment(QtCore.Qt.AlignLeft)
        wrapper_layout.addWidget(ssh_opt)
        wrapper_layout.addWidget(self.ssh)
        wrapper_layout.addStretch()

        self.layout.addWidget(wrapper)

        self.layout.addWidget(self.catkin_ws)
        self.layout.addWidget(self.args)

        self.layout.addWidget(self.ssh_target)
        self.layout.addWidget(self.passwd)
        self.layout.addWidget(self.remote_catkin_ws)

        self.layout.addWidget(self.buttonBox)
        self.layout.addStretch()

        self.setLayout(self.layout)
        self.handle_ssh_toggle()

    def handle_ssh_toggle(self):
        if self.use_ssh:
            self.catkin_ws.setVisible(False)
            self.ssh_target.setVisible(True)
            self.passwd.setVisible(True)
            self.remote_catkin_ws.setVisible(True)
        else:
            self.catkin_ws.setVisible(True)
            self.ssh_target.setVisible(False)
            self.passwd.setVisible(False)
            self.remote_catkin_ws.setVisible(False)
        self.use_ssh = not self.use_ssh

    def create_default_config(self):
        default_config = {
            'catkin_ws': '/path/to/catkin_ws',
            'args': 'args'
        }
        os.makedirs(os.path.dirname(self.config), exist_ok=True)
        with open(self.config, 'w') as file:
            yaml.dump(default_config, file)

    def create_default_ssh_config(self):
        default_config = {
            'ssh_target': 'scandy@10.0.0.250',
            'passwd': 'alex',
            'remote_catkin_ws': '/home/scandy/AD'
        }
        os.makedirs(os.path.dirname(self.ssh_config), exist_ok=True)
        with open(self.ssh_config, 'w') as file:
            yaml.dump(default_config, file)

    def accept(self):
        src_ros = 'source /opt/ros/noetic/setup.sh'
        src_devel = 'source devel/setup.bash'
        if self.args_cached == 'args':
            self.args_cached = ''
        if not self.use_ssh:
            target = self.ssh_target.text()
            passwd = self.passwd.text()
            catkin_ws = self.remote_catkin_ws.text()
            args = self.args.text()
            if not target:
                target = self.ssh_target_cached
            else:
                self.ssh_cache['ssh_target'] = target
            if not passwd:
                passwd = self.passwd_cached
            else:
                self.ssh_cache['passwd'] = passwd
            if not catkin_ws:
                catkin_ws = self.remote_catkin_ws_cached
            else:
                self.ssh_cache['remote_catkin_ws'] = catkin_ws
            if not args:
                args = self.args_cached
            else:
                self.cache['args'] = args
            ssh = f'sshpass -p {passwd} ssh {target}'
            remote_command = f'"{ssh} && {src_ros} && cd {catkin_ws} && {src_devel} && roslaunch control controller.launch {args}"'
            self.cmd = f'{ssh} {remote_command}'
        else:
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
            self.cmd = f'{src_ros} && cd {catkin_ws} && {src_devel} && roslaunch control controller.launch {args}'
        with open(self.ssh_config, 'w') as file:
            yaml.dump(self.ssh_cache, file)
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
