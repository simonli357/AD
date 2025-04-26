from PyQt5.QtWidgets import QVBoxLayout, QHBoxLayout, QLineEdit, QDialog, QDialogButtonBox, QWidget, QLabel, QSpinBox, QToolButton
from PyQt5 import QtCore, QtWidgets


class CustomSpinBox(QSpinBox):
    def __init__(self, attributes, parent=None):
        super().__init__(parent)
        self._attrs = attributes
        self.setRange(0, max(attributes.keys()))
        self.setWrapping(True)
        self.setKeyboardTracking(False)
        self.setButtonSymbols(QSpinBox.NoButtons)

    def textFromValue(self, value: int) -> str:
        return self._attrs.get(value, "")


class AttributeSpinBox(QWidget):
    def __init__(self, attributes, parent=None):
        super().__init__(parent)

        self.spin = CustomSpinBox(attributes, parent)
        self._attrs = attributes

        self.up = QToolButton(self)
        self.up.setText("▲")
        self.up.clicked.connect(self.spin.stepUp)

        self.down = QToolButton(self)
        self.down.setText("▼")
        self.down.clicked.connect(self.spin.stepDown)

        self.label = QWidget(self.spin)

        btn_layout = QVBoxLayout()
        btn_layout.setContentsMargins(0, 0, 0, 0)
        btn_layout.setSpacing(0)
        btn_layout.addWidget(self.up)
        btn_layout.addWidget(self.down)

        main_layout = QHBoxLayout(self)
        main_layout.setContentsMargins(0, 0, 0, 0)
        main_layout.addWidget(self.spin)
        main_layout.addLayout(btn_layout)

        self.setStyleSheet("""
            QToolButton {
                color: white;
                background-color: rgba(255, 255, 255, 0.05);
                border: none;
            }
            QSpinBox {
                color: white;
                border: 1px solid rgba(255,255,255,0.25);
                padding: 10px;
                border-radius: 4px;
            }
        """)

    def setValue(self, v: int):
        self.spin.setValue(v)

    def getValue(self) -> int:
        return self.spin.value()


class NodeFormWidget(QDialog):
    def __init__(self, on_accept, on_delete, node_index, node_id, attr, node_color, x, y):
        super().__init__()
        self.on_accept = on_accept
        self.on_delete = on_delete
        self.node_index = node_index
        self.id = node_id
        self.attr = attr
        self.node_color = f"rgba({node_color[0] * 255}, {node_color[1] * 255}, {node_color[2] * 255}, 1.0)"
        self.x = x
        self.y = y
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMinimumWidth(600)
        self.setStyleSheet("""
            background-color: transparent;
            color: white;
            font-size: 16px;
        """)
        self.ATTRIBUTES_TEXT = {
            0: 'NORMAL',
            1: 'CROSSWALK',
            2: 'INTERSECTION',
            3: 'ONEWAY',
            4: 'HIGHWAY_LEFT',
            5: 'HIGHWAY_RIGHT',
            6: 'ROUNDABOUT',
            7: 'STOPLINE',
            8: 'DOTTED',
            9: 'DOTTED_CROSSWALK',
        }
        self.setup_ui()

    def setup_ui(self):
        title = QLabel(f" {self.id} - {self.ATTRIBUTES_TEXT[self.attr]}")

        title.setAlignment(QtCore.Qt.AlignCenter)
        title.setStyleSheet(f"""
            QLabel {{
                font-size: 32px;
                color: {self.node_color};
                padding: 10px;
            }}
        """)

        self.x_coords = QLineEdit(self)
        self.x_coords.setText(f"{self.x}")
        self.x_coords_label = QLabel(' X')
        self.x_coords_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.y_coords = QLineEdit(self)
        self.y_coords.setText(f"{self.y}")
        self.y_coords_label = QLabel(' Y')
        self.y_coords_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        self.attributes = AttributeSpinBox(self.ATTRIBUTES_TEXT, self)
        self.attributes.setValue(self.attr)
        self.attributes_label = QLabel(' Attribute')
        self.attributes_label.setAlignment(QtCore.Qt.AlignRight | QtCore.Qt.AlignVCenter)

        QBtn = QDialogButtonBox.Ok | QDialogButtonBox.Cancel

        self.buttonBox = QDialogButtonBox(QBtn)
        self.buttonBox.accepted.connect(self.accept)
        self.buttonBox.rejected.connect(self.reject)
        # Get the OK and Cancel buttons
        ok_button = self.buttonBox.button(QDialogButtonBox.Ok)
        cancel_button = self.buttonBox.button(QDialogButtonBox.Cancel)

        # Assign unique object names for styling
        ok_button.setObjectName("okButton")
        cancel_button.setObjectName("cancelButton")
        cancel_button.setText("Delete")

        # Apply styles
        self.buttonBox.setStyleSheet("""
            QPushButton#okButton {
                background-color: #4CAF50;
                color: white;
                border: none;
                padding: 8px 16px;
            }
            QPushButton#cancelButton {
                background-color: #f44336;
                color: white;
                border: none;
                padding: 8px 16px;
            }
            QPushButton:hover {
                opacity: 0.9;
            }
        """)

        self.layout = QVBoxLayout()
        self.layout.setAlignment(QtCore.Qt.AlignTop)
        self.layout.setContentsMargins(10, 10, 10, 10)

        self.layout.addWidget(title)

        labels = QWidget()
        labels_layout = QVBoxLayout(labels)
        labels_layout.setAlignment(QtCore.Qt.AlignLeft)
        labels_layout.addWidget(self.attributes_label)
        labels_layout.addWidget(self.x_coords_label)
        labels_layout.addWidget(self.y_coords_label)

        text_fields = QWidget()
        text_fields_layout = QVBoxLayout(text_fields)
        text_fields_layout.setAlignment(QtCore.Qt.AlignLeft)
        text_fields_layout.addWidget(self.attributes)
        text_fields_layout.addWidget(self.x_coords)
        text_fields_layout.addWidget(self.y_coords)

        form = QWidget()
        form.setStyleSheet("""
            QLineEdit {
                background-color: transparent;
                padding: 10px 10px 10px 10px;
                border: 1px solid rgba(255,255,255,0.25);
                border-radius: 4px;
            }
            QLabel {
                background-color: transparent;
                padding: 10px 10px 10px 10px;
                color: #0099ff;
            }
        """)
        form_layout = QHBoxLayout(form)
        form_layout.addWidget(labels)
        form_layout.addWidget(text_fields)

        self.layout.addStretch()
        self.layout.addWidget(form)
        self.layout.addStretch()
        self.layout.addWidget(self.buttonBox)

        self.setLayout(self.layout)

    def clear_inputs(self):
        self.x_coords.clear()
        self.y_coords.clear()

    def accept(self):
        self.on_accept(self.node_index, float(self.x_coords.text()), float(self.y_coords.text()), self.attributes.getValue())
        self.clear_inputs()
        super().accept()

    def reject(self):
        self.on_delete(self.node_index)
        self.clear_inputs()
        super().reject()
