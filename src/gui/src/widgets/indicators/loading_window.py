from PyQt5.QtWidgets import QDialog, QVBoxLayout
from PyQt5 import QtWidgets

from .loading_spinner import LoadingSpinner


class LoadingWindow(QDialog):
    def __init__(self, process):
        super().__init__()
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMinimumSize(200, 200)
        process.finished.connect(self.close)
        self.setup_ui()
        self.setStyleSheet("background-color: transparent;")

    def setup_ui(self):
        layout = QVBoxLayout()
        layout.addWidget(LoadingSpinner())
        self.setLayout(layout)
