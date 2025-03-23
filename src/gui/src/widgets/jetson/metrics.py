from PyQt5.QtWidgets import QVBoxLayout, QLabel
from PyQt5 import QtWidgets, QtCore


class MetricsWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.parent_window = self.parent()
        self.setup_ui()

    def update_metrics(self):
        self.ram.setText(f'Ram   : <span style="color: green;">{self.parent_window.ram_usage * 100:.0f}%</span>')
        self.temperature.setText(f'Temp  : <span style="color: green;">{self.parent_window.temperature:.1f}°C</span>')
        self.stack.setText(f'Stack  : <span style="color: green;">{self.parent_window.stack_usage * 100:.0f}%</span>')
        self.heap.setText(f'Heap  : <span style="color: green;">{self.parent_window.heap_usage * 100:.0f}%</span>')

    def setup_ui(self):
        self.layout = QVBoxLayout()
        self.layout.setContentsMargins(15, 5, 0, 10)
        self.ram = QLabel(f'Ram   : <span style="color: green;">{self.parent_window.ram_usage * 100:.0f}%</span>')
        self.ram.setStyleSheet("""
            color: orange;
        """)
        self.ram.setAlignment(QtCore.Qt.AlignLeft)
        self.temperature = QLabel(f'Temp  : <span style="color: green;">{self.parent_window.temperature:.1f}°C</span>')
        self.temperature.setStyleSheet("""
            color: orange;
        """)
        self.temperature.setAlignment(QtCore.Qt.AlignLeft)
        self.stack = QLabel(f'Stack  : <span style="color: green;">{self.parent_window.stack_usage * 100:.0f}%</span>')
        self.stack.setStyleSheet("""
            color: orange;
        """)
        self.stack.setAlignment(QtCore.Qt.AlignLeft)
        self.heap = QLabel(f'Heap  : <span style="color: green;">{self.parent_window.heap_usage * 100:.0f}%</span>')
        self.heap.setStyleSheet("""
            color: orange;
        """)
        self.heap.setAlignment(QtCore.Qt.AlignLeft)
        self.layout.addWidget(self.temperature)
        self.layout.addWidget(self.ram)
        self.layout.addWidget(self.stack)
        self.layout.addWidget(self.heap)
        self.setLayout(self.layout)
