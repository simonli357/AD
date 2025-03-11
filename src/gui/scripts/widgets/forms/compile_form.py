from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QLineEdit, QDialog, QDialogButtonBox, QWidget, QLabel
from PyQt5 import QtCore, QtWidgets
from .animated_toggle import AnimatedToggle

import os
import yaml


class CompileFormWidget(QDialog):
    def __init__(self):
        super().__init__()
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMinimumWidth(600)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.data_dir = os.path.join(current_dir, 'appdata')
        self.ssh_config = os.path.join(self.data_dir, 'ssh.yaml')
        self.config = None
        self.setup_ui()
        self.setStyleSheet("""
            background-color: rgba(255, 255, 255, 0.05);
            color: white;
            font-size: 16px;
        """)

    def setup_ui(self):
        self.config = os.path.join(self.data_dir, 'compile.yaml')
        title = QLabel(' compile')

        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet("""
            QLabel {
                font-size: 32px;
                color: orange;
            }
        """)

        self.load_cache()

        self.ssh = AnimatedToggle()
        self.ssh.stateChanged.connect(self.handle_ssh_toggle)

        self.ssh_target = QLineEdit(self)
        self.ssh_target.setText(self.ssh_target_cached)
        self.ssh_target_label = QLabel('SSH Identity  ')
        self.ssh_target_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.passwd = QLineEdit(self)
        self.passwd.setText(self.passwd_cached)
        self.passwd_label = QLabel('SSH Password  ')
        self.passwd_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.catkin_ws = QLineEdit(self)
        self.catkin_ws.setText(self.catkin_ws_cached)
        self.catkin_ws_label = QLabel('Path  ')
        self.catkin_ws_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.remote_catkin_ws = QLineEdit(self)
        self.remote_catkin_ws.setText(self.remote_catkin_ws_cached)
        self.remote_catkin_ws_label = QLabel('Path  ')
        self.remote_catkin_ws_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.cmd = QLineEdit(self)
        self.cmd.setText(self.cmd_cached)
        self.cmd_label = QLabel('CMD ')
        self.cmd_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.remote_cmd = QLineEdit(self)
        self.remote_cmd.setText(self.remote_cmd_cached)
        self.remote_cmd_label = QLabel('CMD ')
        self.remote_cmd_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

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

        self.layout = QVBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        self.layout.addStretch()

        self.layout.addWidget(title)

        ssh_opt = QLabel(" Use SSH")

        wrapper = QWidget()
        wrapper.setStyleSheet("""
            QLabel {
                background-color: transparent;
                color: green;
                font-size: 24px;
                margin-left: 24px;
            }
        """)
        wrapper_layout = QHBoxLayout(wrapper)
        wrapper_layout.setAlignment(QtCore.Qt.AlignLeft)
        wrapper_layout.addWidget(ssh_opt)
        wrapper_layout.addWidget(self.ssh)
        wrapper_layout.addStretch()

        labels = QWidget()
        labels_layout = QVBoxLayout(labels)
        labels_layout.setAlignment(QtCore.Qt.AlignLeft)
        labels_layout.addWidget(self.ssh_target_label)
        labels_layout.addWidget(self.passwd_label)
        labels_layout.addWidget(self.catkin_ws_label)
        labels_layout.addWidget(self.remote_catkin_ws_label)
        labels_layout.addWidget(self.cmd_label)
        labels_layout.addWidget(self.remote_cmd_label)

        text_fields = QWidget()
        text_fields_layout = QVBoxLayout(text_fields)
        text_fields_layout.setAlignment(QtCore.Qt.AlignLeft)
        text_fields_layout.addWidget(self.ssh_target)
        text_fields_layout.addWidget(self.passwd)
        text_fields_layout.addWidget(self.catkin_ws)
        text_fields_layout.addWidget(self.remote_catkin_ws)
        text_fields_layout.addWidget(self.cmd)
        text_fields_layout.addWidget(self.remote_cmd)

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
                color: #0099ff;
            }
        """)
        form_layout = QHBoxLayout(form)
        form_layout.addWidget(labels)
        form_layout.addWidget(text_fields)

        self.layout.addWidget(wrapper)
        self.layout.addWidget(form)

        self.layout.addWidget(self.buttonBox)
        self.layout.addStretch()

        self.setLayout(self.layout)
        self.handle_ssh_toggle()

    def handle_ssh_toggle(self):
        if self.use_ssh:
            self.catkin_ws.setVisible(False)
            self.catkin_ws_label.setVisible(False)
            self.cmd.setVisible(False)
            self.cmd_label.setVisible(False)
            self.ssh_target.setVisible(True)
            self.ssh_target_label.setVisible(True)
            self.passwd.setVisible(True)
            self.passwd_label.setVisible(True)
            self.remote_catkin_ws.setVisible(True)
            self.remote_catkin_ws_label.setVisible(True)
            self.remote_cmd.setVisible(True)
            self.remote_cmd_label.setVisible(True)
        else:
            self.catkin_ws.setVisible(True)
            self.catkin_ws_label.setVisible(True)
            self.cmd.setVisible(True)
            self.cmd_label.setVisible(True)
            self.ssh_target.setVisible(False)
            self.ssh_target_label.setVisible(False)
            self.passwd.setVisible(False)
            self.passwd_label.setVisible(False)
            self.remote_catkin_ws.setVisible(False)
            self.remote_catkin_ws_label.setVisible(False)
            self.remote_cmd.setVisible(False)
            self.remote_cmd_label.setVisible(False)
        self.use_ssh = not self.use_ssh

    def load_cache(self):
        if not os.path.exists(self.data_dir):
            os.makedirs(self.data_dir)
        if not os.path.isfile(self.config):
            self.create_default_config()
        if not os.path.isfile(self.ssh_config):
            self.create_default_ssh_config()

        self.catkin_ws_cached = None
        self.cmd_cached = None
        self.remote_cmd_cached = None
        self.ssh_target_cached = None
        self.passwd_cached = None
        self.remote_catkin_ws_cached = None
        self.cache = None
        self.ssh_cache = None
        self.command = None
        self.use_ssh = False

        with open(self.config, 'r') as file:
            self.cache = yaml.safe_load(file)
            self.catkin_ws_cached = self.cache['catkin_ws']
            self.cmd_cached = self.cache['cmd']
            self.remote_cmd_cached = self.cache['remote_cmd']

        with open(self.ssh_config, 'r') as file:
            self.ssh_cache = yaml.safe_load(file)
            self.ssh_target_cached = self.ssh_cache['ssh_target']
            self.passwd_cached = self.ssh_cache['passwd']
            self.remote_catkin_ws_cached = self.ssh_cache['remote_catkin_ws']

    def create_default_config(self):
        default_config = {
            'catkin_ws': '/path/to/catkin_ws',
            'cmd': 'catkin_make',
            'remote_cmd': 'catkin_make'
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

    def set_remote_cmd(self):
        target = self.ssh_target.text()
        passwd = self.passwd.text()
        catkin_ws = self.remote_catkin_ws.text()
        cmd = self.remote_cmd.text()
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
        if not cmd:
            cmd = self.remote_cmd_cached
        else:
            self.cache['remote_cmd'] = cmd
        bash = 'bash -ic'
        src_ros = 'source /opt/ros/noetic/setup.sh'
        src_devel = f'source {catkin_ws}/devel/setup.bash'
        ssh = f'sshpass -p {passwd} ssh -tt {target}'
        remote_command = f'\'exec bash -c "{src_ros} && {src_devel} && {cmd}"\''
        self.command = f'{ssh} "{bash} {remote_command}"'

    def set_local_cmd(self):
        src_ros = 'source /opt/ros/noetic/setup.sh'
        src_devel = 'source devel/setup.bash'
        catkin_ws = self.catkin_ws.text()
        cmd = self.cmd.text()
        if not catkin_ws:
            catkin_ws = self.catkin_ws_cached
        else:
            self.cache['catkin_ws'] = catkin_ws
        if not cmd:
            cmd = self.cmd_cached
        else:
            self.cache['cmd'] = cmd
        self.command = f'{src_ros} && cd {catkin_ws} && {src_devel} && {cmd}'

    def clear_inputs(self):
        self.catkin_ws.clear()
        self.cmd.clear()

    def accept(self):
        if self.args_cached == 'args':
            self.args_cached = ''
        if not self.use_ssh:
            self.set_remote_cmd()
        else:
            self.set_local_cmd()
        with open(self.ssh_config, 'w') as file:
            yaml.dump(self.ssh_cache, file)
        with open(self.config, 'w') as file:
            yaml.dump(self.cache, file)
        self.clear_inputs()
        super().accept()

    def reject(self):
        self.clear_inputs()
        super().reject()

    def get_cmd(self):
        cmd = self.command
        self.command = None
        return cmd
