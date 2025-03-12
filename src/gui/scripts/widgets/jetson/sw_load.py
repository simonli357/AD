from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget
from PyQt5 import QtCore, QtWidgets
from .cores import CoresWidget
from .icon import JetsonIcon
# from .metrics import MetricsWidget


class SoftwareMetricsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.cores_usage = [0.5, 0.5, 0.5, 0.5, 0.5, 0.5]
        self.ram_usage = 0.0
        self.temperature = 0.0
        self.heap_usage = 0.0
        self.stack_usage = 0.0

        self.cores_widget = CoresWidget(self)
        self.jetson_icon_widget = JetsonIcon()
        # self.metrics_widget = MetricsWidget()
        self.setup_ui()
        self.setStyleSheet("""
            color: white;
            font-size: 16px;
        """)

    def set_load(self, msg):
        self.cores_usage = msg.cores_usage
        self.ram_usage = msg.ram_usage
        self.temperature = msg.temperature
        self.heap_usage = msg.heap_usage
        self.stack_usage = msg.stack_usage

    def render_widget(self):
        self.cores_widget.update_usages()

    def setup_ui(self):
        self.layout = QVBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignCenter)
        top_wrapper = QWidget()
        top_wrapper_layout = QHBoxLayout(top_wrapper)
        top_wrapper_layout.setAlignment(QtCore.Qt.AlignCenter)
        # top_wrapper_layout.addWidget(self.metrics_widget)
        top_wrapper_layout.addStretch()
        top_wrapper_layout.addWidget(self.jetson_icon_widget)

        self.layout.addWidget(top_wrapper)
        self.layout.addWidget(self.cores_widget)

        self.setLayout(self.layout)
