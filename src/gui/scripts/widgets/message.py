from PyQt5 import QtWidgets, QtCore
from collections import deque


class MessageWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.message_history = deque([], 1000)
        self.setMaximumHeight(250)
        self.setup_ui()

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QHBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignJustify)
        self.message_display = QtWidgets.QTextEdit()
        self.message_display.setReadOnly(True)

        self.message_display.setStyleSheet("""
            QTextEdit {
                background-color: rgba(255, 255, 255, 0.1);
                color: #00FF00;
                margin: 12px;
                font-family: 'Roboto';
                font-size: 24px;
                border-radius: 12px;
                padding: 5px;
            }
        """)
        self.layout.addWidget(self.message_display)

    def add_message(self, message) -> None:
        self.message_history.append(message)
        timestamp = QtCore.QDateTime.currentDateTime().toString("[hh:mm:ss] ")
        self.message_display.setPlainText("\n".join(
            [f"{timestamp}{msg}" for msg in self.message_history]
        ))
        self.message_display.verticalScrollBar().setValue(
            self.message_display.verticalScrollBar().maximum()
        )
