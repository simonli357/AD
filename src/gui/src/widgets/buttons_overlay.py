from PyQt5.QtWidgets import QWidget, QHBoxLayout, QPushButton, QLabel, QMenu

import os
import requests
import xml.etree.ElementTree as ET


class CommentedTreeBuilder(ET.TreeBuilder):
    def comment(self, data):
        self.start(ET.Comment, {})
        self.data(data)
        self.end(ET.Comment)


class ButtonsOverlay(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.runs = []
        self.read_from_cache()
        self.setup_ui()
        self.wrapper.update()
        self.adjustSize()

    def read_from_cache(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        main_dir = os.path.dirname(current_dir)
        runs_file = os.path.join(main_dir, 'runs.xml')
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

    def setup_ui(self):
        self.wrapper = QHBoxLayout(self)

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

        self.ip_label = QLabel("  ---.---.-.-")
        self.ip_label.setStyleSheet("""
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

        self.measuring_btn = QPushButton("󰭍")
        self.measuring_btn.clicked.connect(self.handle_measuring_clicked)

        self.cam_lock_btn = QPushButton("")
        self.cam_lock_btn.clicked.connect(self.toggle_cam_lock)

        self.wrapper.addWidget(self.run_label)
        self.wrapper.addWidget(self.ip_label)
        self.wrapper.addWidget(self.change_run_btn)
        self.wrapper.addWidget(self.swap_map_btn)
        self.wrapper.addWidget(self.measuring_btn)
        self.wrapper.addWidget(self.cam_lock_btn)

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

        self.update_button_style(self.measuring_btn, self.main_window.map_widget.measuring)
        self.update_button_style(self.cam_lock_btn, self.main_window.map_widget.cam_locked)

        self.fetch_ip()

    def update_button_style(self, button, is_active):
        """Update button color based on boolean state"""
        color = "#ffcc00" if is_active else "rgba(255, 255, 255, 0.08);"  # Light green/red
        button.setStyleSheet(f"""
            QPushButton {{
                color: white;
                border: none;
                font-size: 24px;
                font-weight: bold;
                background-color: rgba(255, 255, 255, 0.08);
                border-radius: 8px;
                padding: 5px 12px 5px 10px;
                background-color: {color};
            }}
            QPushButton:hover {{
                background-color: #e6e600;
            }}
        """)

    def set_run_name(self, name: str) -> None:
        self.run_label.setText(f' {name}')
        self.wrapper.update()
        self.adjustSize()

    def set_ip(self, ip: str) -> None:
        self.ip_label.setText(f' {ip}')
        self.wrapper.update()
        self.adjustSize()

    def fetch_ip(self):
        url = "https://ip-broadcaster-worker.yu-qing-liu.workers.dev"
        try:
            response = requests.get(url).text
            self.set_ip(response.rstrip('%').strip())
        except Exception as e:
            print("Failed to get IP:", e)

    def show_menu(self) -> None:
        original_point = self.change_run_btn.mapToGlobal(self.change_run_btn.rect().topRight())
        self.menu.exec_(original_point)

    def swap_map(self) -> None:
        self.main_window.toggle_map()

    def handle_measuring_clicked(self) -> None:
        self.main_window.map_widget.measuring = not self.main_window.map_widget.measuring
        if not self.main_window.map_widget.measuring:
            self.main_window.map_widget.click_history.clear()
        else:
            self.main_window.map_widget.cursor_coords.clear()
        self.update_button_style(self.measuring_btn, self.main_window.map_widget.measuring)

    def handle_cam_lock_clicked(self, locked) -> None:
        self.main_window.map_widget.cam_locked = locked
        self.update_button_style(self.cam_lock_btn, locked)

    def toggle_cam_lock(self) -> None:
        self.main_window.map_widget.cam_locked = not self.main_window.map_widget.cam_locked
        self.update_button_style(self.cam_lock_btn, self.main_window.map_widget.cam_locked)

    def update_launch_params(self, x0, y0, yaw0, path):
        pass

    def on_action_triggered(self, run):
        x0 = run.get('x0')
        y0 = run.get('y0')
        yaw0 = run.get('yaw0')
        path = run.get('path')
        self.update_launch_params(x0, y0, yaw0, path)
