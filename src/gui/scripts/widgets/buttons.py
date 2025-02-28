from PyQt5 import QtWidgets, QtCore
from collections import deque


class ButtonsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setup_ui()

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.buttons = deque()

        self.start_btn = QtWidgets.QPushButton("")
        self.goto_btn = QtWidgets.QPushButton("󰓾")

        self.buttons.append(self.start_btn)
        self.buttons.append(self.goto_btn)

        for btn in self.buttons:
            self.layout.addWidget(btn)

        self.layout.addStretch()

        self.setStyleSheet("""
            QPushButton {
                background-color: rgba(255, 255, 255, 0.15);
                padding: 12px 36px 12px 32px;
                margin-right: 12px;
                color: white;
                border: none;
                border-radius: 5px;
                font-size: 32px;
            }
            QPushButton:hover {
                background-color: #2980b9;
            }
            QPushButton:pressed {
                background-color: #1c6da8;
            }
        """)
