from PyQt5.QtGui import QPainter, QPen, QColor
from PyQt5 import QtCore, QtWidgets

import numpy as np


class ObjectWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.main_window = self.parent()

        self.obj_dict = self.main_window.map_widget.object_dict

        # Real-world extents (meters)
        self.x_min, self.x_max = -1.5, 1.5  # X-axis range
        self.y_min, self.y_max = -1.5, 1.5  # Y-axis range

    def add_object(self, obj):
        self.objects.append(obj)

    def render_widget(self) -> None:
        self.update()

    def paintEvent(self, event):
        with QPainter(self) as painter:
            painter.setRenderHint(QPainter.Antialiasing)
            dimensions = self.draw_axis(painter)
            h1, h2, w1, w2, mid_h, mid_w = dimensions
            # Calculate pixels per meter based on current widget size and real-world extents
            try:
                pixels_per_meter_x = (w2 - w1) / (self.x_max - self.x_min)
                pixels_per_meter_y = (h2 - h1) / (self.y_max - self.y_min)
            except ZeroDivisionError:
                return  # Avoid division by zero if extents are not set properly
            self.draw_grid(painter, h1, h2, w1, w2, mid_w, pixels_per_meter_x, pixels_per_meter_y)
            self.draw_detected_object(painter, mid_h, mid_w, h2, pixels_per_meter_x, pixels_per_meter_y)

    def draw_axis(self, painter: QPainter) -> tuple:
        # Calculate axis positions
        h1 = 0.0
        h2 = self.height()
        w1 = 0.0
        w2 = self.width()
        mid_h = self.height() / 2
        mid_w = self.width() / 2

        # Draw vertical axis (Y-axis)
        painter.setPen(QPen(QColor(0, 255, 0), 1))
        painter.drawLine(QtCore.QPointF(mid_w, h1), QtCore.QPointF(mid_w, h2))

        # Draw horizontal axis (X-axis)
        painter.setPen(QPen(QColor(255, 0, 0), 1))
        painter.drawLine(QtCore.QPointF(w1, mid_h), QtCore.QPointF(w2, mid_h))

        return (h1, h2, w1, w2, mid_h, mid_w)

    def draw_grid(self, painter: QPainter, h1: float, h2: float, w1: float, w2: float, mid_w: float, pixels_per_meter_x: float, pixels_per_meter_y: float) -> None:
        # Draw vertical grid lines (X-axis)
        painter.setPen(QPen(QColor(100, 100, 100, 180), 1))
        x = mid_w + pixels_per_meter_x / 2
        while x <= w2:
            painter.drawLine(QtCore.QPointF(x, h1), QtCore.QPointF(x, h2))
            x += pixels_per_meter_x / 2
        x = mid_w - pixels_per_meter_x / 2
        while x >= w1:
            painter.drawLine(QtCore.QPointF(x, h1), QtCore.QPointF(x, h2))
            x -= pixels_per_meter_x / 2

        # Draw horizontal grid lines (Y-axis)
        y = h2 - pixels_per_meter_y / 2
        while y > h1:
            painter.drawLine(QtCore.QPointF(w1, y), QtCore.QPointF(w2, y))
            y -= pixels_per_meter_y / 2

    def draw_detected_object(self, painter: QPainter, mid_h: float, mid_w: float, h2: float, pixels_per_meter_x: float, pixels_per_meter_y: float):
        obj = self.main_window.map_widget.detected_objects
        # Convert real-world coordinates to widget coordinates
        if obj is None or (obj[8] == 0 and obj[7] == 0):
            return
        widget_x = mid_w - obj[8] * pixels_per_meter_x
        widget_y = mid_h - obj[7] * pixels_per_meter_y

        painter.setPen(QPen(QColor(0, 255, 255), 5))
        painter.drawEllipse(QtCore.QPointF(widget_x, widget_y), 2, 2)

        text_rec = QtCore.QRectF(
            widget_x,
            widget_y - 20,
            80,
            40
        )

        painter.setPen(QPen(QtCore.Qt.yellow, 2))
        painter.drawText(text_rec, QtCore.Qt.AlignCenter, f"x: {obj[7]:.2f} \n y: {obj[8]:.2f}")
