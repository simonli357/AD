from PyQt5.QtWidgets import QHBoxLayout, QVBoxLayout, QWidget, QLabel
from PyQt5 import QtCore, QtWidgets


class CoresWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMaximumWidth(298)
        self.parent_window = self.parent()
        self.cores_progress_bars = []
        self.cores_text = []
        self.setup_ui()
        self.create_progress_bars()
        self.setStyleSheet("""
            CoresWidget {
                background: transparent;
            }
            QLabel {
                color: orange;
                font-size: 12px;
            }
            QProgressBar {
                border: 1px solid rgba(255, 255, 255, 0.3);
                border-radius: 3px;
                background: rgba(255, 255, 255, 0.1);
            }
            QProgressBar::chunk {
                background: qlineargradient(
                    spread:pad, x1:0.5, y1:0, x2:0.5, y2:1,
                    stop:0 rgba(100, 255, 150, 0.8),
                    stop:1 rgba(50, 200, 100, 0.8)
                );
                border-radius: 2px;
            }
        """)

    def create_progress_bars(self):
        # Clear existing widgets
        while self.bar_wrapper_layout.count():
            item = self.bar_wrapper_layout.takeAt(0)
            if item.widget():
                item.widget().deleteLater()
        self.cores_progress_bars.clear()

        # Create new bars
        num_bars = 0
        for use in self.parent_window.cores_usage:
            if num_bars == 6:
                return
            pbar = QtWidgets.QProgressBar()
            pbar.setRange(0, 100)
            pbar.setValue(int(use * 100))
            pbar.setOrientation(QtCore.Qt.Vertical)
            pbar.setMaximumWidth(15)
            pbar.setTextVisible(False)

            # Create wrapper with centered elements
            wrapper = QWidget()
            wrapper_layout = QVBoxLayout(wrapper)
            wrapper_layout.setContentsMargins(0, 0, 0, 0)
            wrapper_layout.setSpacing(2)
            wrapper_layout.setAlignment(QtCore.Qt.AlignCenter)

            # Centered label
            text = QLabel(f' {use * 100:.0f}%')
            text.setAlignment(QtCore.Qt.AlignCenter)

            # Centered progress bar container
            bar_container = QWidget()
            bar_layout = QHBoxLayout(bar_container)
            bar_layout.setContentsMargins(0, 0, 0, 0)
            bar_layout.addStretch()
            bar_layout.addWidget(pbar)
            bar_layout.addStretch()

            wrapper_layout.addWidget(text)
            wrapper_layout.addWidget(bar_container)
            self.bar_wrapper_layout.addWidget(wrapper)
            self.cores_progress_bars.append(pbar)
            self.cores_text.append(text)
            num_bars += 1

    def update_usages(self):
        num_bars = 0
        for pbar, use in zip(self.cores_progress_bars, self.parent_window.cores_usage):
            if num_bars == 6:
                return
            pbar.setValue(int(use * 100))
            self.cores_text[num_bars].setText(f' {use * 100:.0f}%')
            num_bars += 1

    def setup_ui(self):
        self.layout = QHBoxLayout()
        self.layout.setContentsMargins(0, 0, 0, 0)

        # Main container with centered horizontal layout
        bar_wrapper = QWidget()
        self.bar_wrapper_layout = QHBoxLayout(bar_wrapper)
        self.bar_wrapper_layout.setContentsMargins(0, 0, 0, 0)
        self.bar_wrapper_layout.setSpacing(5)
        self.bar_wrapper_layout.addStretch()

        self.layout.addWidget(bar_wrapper)
        self.setLayout(self.layout)
