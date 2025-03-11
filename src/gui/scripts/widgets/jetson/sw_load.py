from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QWidget
from PyQt5 import QtCore, QtWidgets
# from .cores import CoresWidget
from .icon import JetsonIcon
# from .metrics import MetricsWidget


class SoftwareMetricsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        # self.cores_widget = CoresWidget()
        self.jetson_icon_widget = JetsonIcon()
        # self.metrics_widget = MetricsWidget()
        self.setup_ui()
        self.setStyleSheet("""
            color: white;
            font-size: 16px;
        """)

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
        # self.layout.addWidget(self.metrics_widget)
        self.layout.addStretch()

        self.setLayout(self.layout)
