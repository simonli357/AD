from PyQt5 import QtCore, QtWidgets
from PyQt5.QtWidgets import QLabel, QWidget


class InfoWidget(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.main_window = self.parent()
        self.setup_ui()

    def setup_ui(self):
        self.layout = QtWidgets.QVBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignTop | QtCore.Qt.AlignLeft)
        self.object_type_label = QLabel("󰮄  --:--")
        self.object_type_label.setStyleSheet("""
            border: none;
            background-color: transparent;
            color: yellow;
            font-size: 18px;
        """)
        self.dist_label = QLabel("  --:-- cm")
        self.dist_label.setStyleSheet("""
            border: none;
            background-color: transparent;
            color: yellow;
            font-size: 18px;
        """)
        self.pos_label = QLabel("󰵉  x: --:-- y: --:--")
        self.pos_label.setStyleSheet("""
            border: none;
            background-color: transparent;
            color: yellow;
            font-size: 18px;
        """)

        self.layout.addWidget(self.object_type_label)
        self.layout.addWidget(self.dist_label)
        self.layout.addWidget(self.pos_label)
        self.setLayout(self.layout)

    def update_obj_type(self, obj_type=None):
        if obj_type is None:
            self.object_type_label.setText("󰮄  --:--")
        else:
            self.object_type_label.setText(f"󰮄  {obj_type}")

    def update_obj_dist(self, dist=None):
        if dist is None:
            self.dist_label.setText("  --:-- m")
        else:
            self.dist_label.setText(f"  {dist:.2f} cm")

    def update_obj_pos(self, pos=None):
        if pos is None:
            self.pos_label.setText("󰵉  x: --:-- y: --:--")
        else:
            self.pos_label.setText(f"󰵉  x: {pos[0]:.2f} y: {pos[1]:.2f}")
