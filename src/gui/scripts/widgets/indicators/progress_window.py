from PyQt5.QtWidgets import QDialog, QVBoxLayout
from PyQt5 import QtWidgets


class ProgressWindow(QDialog):
    def __init__(self):
        super().__init__()
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMinimumWidth(600)
        self.progress_bar = QtWidgets.QProgressBar()
        self.progress_bar.setRange(0, 1000)
        self.progress_bar.setValue(0)
        self.setup_ui()
        self.setStyleSheet("""
            background-color: rgba(255, 255, 255, 0.05);
            color: white;
            font-size: 16px;
        """)

    def setup_ui(self):
        layout = QVBoxLayout()
        layout.addWidget(self.progress_bar)

        self.setLayout(layout)

    def increment(self, amount):
        self.progress_bar.setValue(self.progress_bar.value() + amount)

    def set_progress(self, progress):
        if progress * 10 > self.progress_bar.value():
            self.progress_bar.setValue(progress * 10)
            if self.progress_bar.value() == 1000:
                self.close()
