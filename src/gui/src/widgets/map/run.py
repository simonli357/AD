from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton, QSizePolicy, QLabel, QMenu, QDialog, QDialogButtonBox, QVBoxLayout
from PyQt5 import QtCore
from geometry_msgs.msg import PoseWithCovarianceStamped

import os
import xml.etree.ElementTree as ET
import yaml
import tf.transformations as tft
import sys
import numpy as np


class CommentedTreeBuilder(ET.TreeBuilder):
    def comment(self, data):
        self.start(ET.Comment, {})
        self.data(data)
        self.end(ET.Comment)


class ConfirmUpdate(QDialog):
    def __init__(self, run_name=None):
        super().__init__()
        self.run_name = run_name
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setup_ui()
        self.setStyleSheet("""
            background-color: rgba(255, 255, 255, 0.05);
            color: green;
            font-size: 24px;
        """)

    def setup_ui(self):
        QBtn = QDialogButtonBox.Ok
        self.buttonBox = QDialogButtonBox(QBtn)
        ok_button = self.buttonBox.button(QDialogButtonBox.Ok)
        ok_button.setObjectName("okButton")
        self.buttonBox.setStyleSheet("""
            QPushButton#okButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px 16px;
            }
            QPushButton:hover {
                opacity: 0.9;
            }
        """)
        self.buttonBox.accepted.connect(self.accept)

        self.info = QLabel(f'Injected {self.run_name} into controller launchfile.')
        self.info.setStyleSheet("""
            background-color: transparent;
            padding-bottom: 5px;
        """)
        self.info2 = QLabel('Restart Controller and Simulator to apply changes.')
        self.info2.setStyleSheet("""
            background-color: transparent;
            padding-bottom: 20px;
        """)

        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(20, 20, 20, 20)
        self.layout.setAlignment(QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.info)
        self.layout.addWidget(self.info2)
        self.layout.addWidget(self.buttonBox)
        self.setLayout(self.layout)

    def accept(self):
        self.close()


class RunOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setMinimumSize(300, 60)
        self.main_window = self.parent()
        self.runs = []
        self.launchfile = None
        self.sim_config = None
        self.sim_file = None
        self.read_from_cache()
        self.setup_ui()

    def create_default_config(self, config):
        default_config = {
            'catkin_ws': '/path/to/catkin_ws',
            'args': 'args'
        }
        os.makedirs(os.path.dirname(config), exist_ok=True)
        with open(config, 'w') as file:
            yaml.dump(default_config, file)

    def read_from_cache(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        main_dir = os.path.dirname(os.path.dirname(current_dir))
        runs_file = os.path.join(main_dir, 'runs.xml')
        controller_file = os.path.join(main_dir, 'widgets', 'forms', 'appdata', 'controller.yaml')
        self.sim_file = os.path.join(main_dir, 'widgets', 'forms', 'appdata', 'simulator.yaml')
        if not os.path.isfile(self.sim_file):
            self.create_default_config(self.sim_file)
        if not os.path.isfile(controller_file):
            self.create_default_config(controller_file)
        with open(self.sim_file, 'r') as file:
            self.sim_config = yaml.safe_load(file)
        with open(controller_file, 'r') as file:
            cache = yaml.safe_load(file)
            catkin_ws = cache['catkin_ws']
            self.launchfile = f'{catkin_ws}/src/control/launch/controller.launch'
        tree = ET.parse(runs_file)
        root = tree.getroot()
        self.runs = []
        current_run = {}
        for arg in root.findall('.//arg'):
            name = arg.get('name')
            default = arg.get('default')
            if name in ['x0', 'y0', 'yaw0', 'path']:
                current_run[name] = default
            if name == 'path':
                self.runs.append(current_run)
                current_run = {}

    def update_launch_params(self, x0, y0, yaw0, path):
        try:
            parser = ET.XMLParser(target=CommentedTreeBuilder())
            tree = ET.parse(self.launchfile, parser)
            root = tree.getroot()
            for arg in root.findall('.//arg'):
                name = arg.get('name')
                if name == 'x0':
                    arg.set('default', x0)
                elif name == 'y0':
                    arg.set('default', y0)
                elif name == 'yaw0':
                    arg.set('default', yaw0)
                elif name == 'path':
                    arg.set('default', path)
            with open(self.launchfile, 'wb') as f:
                self.serialize_xml(root, f)
        except Exception as e:
            print(f"Failed to update launch file: {e}")

    def serialize_xml(self, elem, file, indent=0):
        if elem.tag is ET.Comment:
            file.write(b'  ' * indent + b'<!--' + elem.text.encode() + b'-->\n')
            return
        file.write(b'  ' * indent + b'<' + elem.tag.encode())
        for name, value in elem.attrib.items():
            file.write(b' %s="%s"' % (name.encode(), value.encode()))
        if len(elem) or elem.text:
            file.write(b'>')
            if elem.text:
                file.write(elem.text.encode())
            file.write(b'\n')
            for child in elem:
                self.serialize_xml(child, file, indent + 1)
            file.write(b'  ' * indent + b'</' + elem.tag.encode() + b'>\n')
        else:
            file.write(b'/>\n')

    def setup_ui(self):
        self.run_wrapper = QHBoxLayout(self)
        self.run_label = QLabel("--:--")
        self.run_label.setStyleSheet("""
            color: white;
            border: none;
            font-size: 24px;
            font-weight: bold;
            background-color: rgba(255, 255, 255, 0.08);
            border-radius: 8px;
            padding: 5px 20px 5px 20px;
        """)
        self.change_run_btn = QPushButton("")
        self.change_run_btn.setStyleSheet("""
            QPushButton {
                color: white;
                border: none;
                font-size: 24px;
                font-weight: bold;
                background-color: rgba(255, 255, 255, 0.08);
                border-radius: 8px;
                padding: 5px 18px 5px 10px;
            }
            QPushButton:hover {
                background-color: rgba(255, 255, 255, 0.2);
            }
        """)
        self.change_run_btn.clicked.connect(self.show_menu)
        self.swap_map_btn = QPushButton("  󰓡")
        self.swap_map_btn.setStyleSheet("""
            QPushButton {
                color: white;
                border: none;
                font-size: 24px;
                font-weight: bold;
                background-color: rgba(255, 255, 255, 0.08);
                border-radius: 8px;
                padding: 5px 12px 5px 10px;
            }
            QPushButton:hover {
                background-color: rgba(255, 255, 255, 0.2);
            }
        """)
        self.swap_map_btn.clicked.connect(self.swap_map)

        self.run_wrapper.addWidget(self.run_label)
        self.run_wrapper.addWidget(self.change_run_btn)
        self.run_wrapper.addWidget(self.swap_map_btn)

        self.menu = QMenu(self)
        self.menu.setStyleSheet("""
            QMenu {
                color: white;
                font-size: 16px;
                border: none;
                background-color: transparent;
            }
            QMenu::item {
                background-color: rgba(40, 40, 40, 0.5);
                margin: 1px;
                padding-left: 30px;
                padding-right: 30px;
                padding-top: 4px;
                padding-bottom: 4px;
                border-radius: 8px;
            }
            QMenu::item:selected {
                background-color: purple;
            }
        """)
        for run in self.runs:
            path_name = run.get('path', 'Unknown')
            self.menu.addAction(path_name, lambda checked=False, r=run: self.on_action_triggered(r))

    def set_run_name(self, name: str) -> None:
        self.run_label.setText(f' {name}')
        self.run_wrapper.update()
        self.adjustSize()

    def show_menu(self) -> None:
        original_point = self.change_run_btn.mapToGlobal(self.change_run_btn.rect().topRight())
        self.menu.exec_(original_point)

    def swap_map(self) -> None:
        self.main_window.toggle_map()

    def on_action_triggered(self, run):
        x0 = run.get('x0')
        y0 = run.get('y0')
        yaw0 = run.get('yaw0')
        path = run.get('path')
        self.update_launch_params(x0, y0, yaw0, path)
        with open(self.sim_file, 'w') as file:
            self.sim_config['args'] = f'{path}.launch'
            yaml.dump(self.sim_config, file)
        ConfirmUpdate(path).exec()

    def match_run(self, gps_msg: PoseWithCovarianceStamped) -> None:
        x0_gps = gps_msg.pose.pose.position.x
        y0_gps = gps_msg.pose.pose.position.y
        q = gps_msg.pose.pose.orientation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        quaternion = [qx, qy, qz, qw]
        roll, pitch, yaw_gps = tft.euler_from_quaternion(quaternion)

        runs_with_info = []
        for run in self.runs:
            x0, y0, yaw0, path = run.get('x0'), run.get('y0'), run.get('yaw0'), run.get('path')
            dx = x0 - x0_gps
            dy = y0 - y0_gps
            dist_from_target = np.hypot(dx, dy)
            yaw_diff = abs(yaw0 - yaw_gps)
            runs_with_info.append(((x0, y0, yaw0, path), dist_from_target, yaw_diff))

        # Sort by lowest dist from target, then by lowest yaw_diff
        runs_with_info.sort(key=lambda r: (r[1], r[2]))

        self.main_window.server.utility_node_client.send_gps_msg(*runs_with_info[0][0])
