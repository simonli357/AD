from PyQt5.QtGui import QPainter, QPen, QColor, QPolygonF, QConicalGradient
from PyQt5 import QtCore, QtWidgets
import math


class MeterWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.speed = 0
        self.min_speed = 0.0
        self.max_speed = 70.0
        self.yaw = 0
        self.min_yaw = -180.0
        self.max_yaw = 180.0
        self.center1 = None
        self.center2 = None
        self.size1 = None
        self.size2 = None
        self.angles1 = None
        self.angles2 = None

    def set_speed(self, speed: float) -> None:
        self.speed = max(self.min_speed, min(speed, self.max_speed))
        self.update()

    def set_yaw(self, yaw: float) -> None:
        self.yaw = max(self.min_yaw, min(yaw, self.max_yaw))
        self.update()

    def paintEvent(self, event):
        with QPainter(self) as painter:
            painter.setRenderHint(QPainter.Antialiasing)
            self.draw_meters(painter)
            self.draw_needle_left(painter, self.center1, self.size1, self.angles1, self.speed, self.min_speed, self.max_speed)
            self.draw_needle_right(painter, self.center2, self.size2, self.angles2, self.yaw, self.min_yaw, self.max_yaw)
            self.draw_border(painter)

    def draw_border(self, painter: QPainter) -> None:
        border_color = QColor(0, 255, 255)  # Cyan
        border_width = 4
        border_radius = 10
        painter.setPen(QPen(border_color, border_width))
        painter.setBrush(QtCore.Qt.NoBrush)
        painter.drawRoundedRect(self.rect(), border_radius, border_radius)

    def draw_meters(self, painter: QPainter) -> None:
        widget_width = self.width()
        widget_height = self.height()
        min_length = min(widget_width, widget_height)
        main_radius = min_length - 25
        secondary_radius = main_radius / 1.35
        center1_y = min_length / 1.6
        center2_y = center1_y + ((main_radius - secondary_radius)) / 4
        center1_x = widget_width / 2 - main_radius / 2 + secondary_radius / 3.5
        center2_x = center1_x + secondary_radius

        self.center1 = QtCore.QPointF(center1_x, center1_y)
        self.center2 = QtCore.QPointF(center2_x, center2_y)
        self.size1 = QtCore.QSizeF(main_radius, main_radius)
        self.size2 = QtCore.QSizeF(secondary_radius, secondary_radius)

        self.angles1 = self.draw_speed_ticks(15, painter, self.center1, self.size1, 0.2, -10, 210)
        self.angles1.reverse()
        self.draw_speed_increments(painter, self.center1, self.size1, self.angles1, self.min_speed, self.max_speed)
        self.draw_arc(f'SPEED \n {self.speed:.2f} cm/s', painter, self.center1, self.size1, -20, 220)

        self.angles2 = self.draw_yaw_ticks(7, painter, self.center2, self.size2, 0.2, -10, 140)
        self.draw_yaw_increments(painter, self.center2, self.size2, self.angles2, self.min_yaw, self.max_yaw)
        self.draw_arc(f'YAW \n {self.yaw:.2f} deg', painter, self.center2, self.size2, -20, 150)

    from PyQt5.QtGui import QConicalGradient

    def draw_inner(self, painter: QPainter, center: QtCore.QPointF, size: QtCore.QSizeF, start_angle: int, span_angle: int) -> None:
        s = size * 0.97
        rect = QtCore.QRectF(
            center.x() - s.width() / 2,
            center.y() - s.height() / 2,
            s.width(),
            s.height()
        )
        gradient = QConicalGradient(center, start_angle - 5)
        gradient.setColorAt(0.00, QColor(255, 0, 0))    # Red
        gradient.setColorAt(0.33, QColor(255, 255, 0))  # Yellow
        gradient.setColorAt(0.66, QColor(0, 255, 0))    # Green
        painter.setPen(QPen(gradient, 4))
        painter.drawArc(rect, start_angle * 16, span_angle * 16)

    def draw_arc(self, title: str, painter: QPainter, center: QtCore.QPointF, size: QtCore.QSizeF, start_angle: int, span_angle: int) -> None:
        rect = QtCore.QRectF(
            center.x() - size.width() / 2,
            center.y() - size.height() / 2,
            size.width(),
            size.height()
        )
        painter.setPen(QPen(QtCore.Qt.lightGray, 3))
        painter.drawArc(rect, start_angle * 16, span_angle * 16)
        self.draw_inner(painter, center, size, start_angle, span_angle)

        font = painter.font()
        font.setPointSize(10)
        font.setFamily("Arial")
        font.setBold(True)
        painter.setFont(font)
        painter.setPen(QPen(QtCore.Qt.lightGray, 2))

        text_width = painter.fontMetrics().width(title)
        text_height = painter.fontMetrics().height()

        text_rect = QtCore.QRectF(
            center.x() - text_width / 2,
            center.y() + 15,
            text_width,
            text_height * 2
        )

        painter.drawText(text_rect, QtCore.Qt.AlignCenter, title)

    def draw_speed_ticks(self, num_ticks: int, painter: QPainter, center: QtCore.QSizeF, size: QtCore.QSizeF, tick_length: float, start_angle: int, span_angle: int) -> list:
        total_angle = span_angle - start_angle
        step = total_angle / num_ticks
        angles = [start_angle + i * step for i in range(num_ticks)]

        painter.setPen(QPen(QColor(255, 255, 255), 1))

        radius_x = size.width() / 2
        radius_y = size.height() / 2

        i = 0
        for angle in angles:
            angle_rad = math.radians(angle)

            outer_point = QtCore.QPointF(
                center.x() + radius_x * math.cos(angle_rad),
                center.y() - radius_y * math.sin(angle_rad)
            )

            length = tick_length / 2 if i % 2 != 0 else tick_length

            inner_point = QtCore.QPointF(
                center.x() + (radius_x * (1.0 - length)) * math.cos(angle_rad),
                center.y() - (radius_y * (1.0 - length)) * math.sin(angle_rad)
            )

            painter.drawLine(outer_point, inner_point)
            i += 1

        return angles

    def draw_yaw_ticks(self, num_ticks: int, painter: QPainter, center: QtCore.QSizeF, size: QtCore.QSizeF, tick_length: float, start_angle: int, span_angle: int) -> list:
        total_angle = span_angle - start_angle
        step = total_angle / num_ticks
        angles = [start_angle + i * step for i in range(num_ticks)]

        painter.setPen(QPen(QColor(255, 255, 255), 1))

        radius_x = size.width() / 2
        radius_y = size.height() / 2

        for angle in angles:
            angle_rad = math.radians(angle)

            outer_point = QtCore.QPointF(
                center.x() + radius_x * math.cos(angle_rad),
                center.y() - radius_y * math.sin(angle_rad)
            )

            inner_point = QtCore.QPointF(
                center.x() + (radius_x * (1.0 - tick_length)) * math.cos(angle_rad),
                center.y() - (radius_y * (1.0 - tick_length)) * math.sin(angle_rad)
            )

            painter.drawLine(outer_point, inner_point)

        return angles

    def draw_speed_increments(self, painter: QPainter, center: QtCore.QSizeF, size: QtCore.QSizeF, angles: list, min_value: float, max_value: float) -> None:
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        painter.setPen(QPen(QtCore.Qt.lightGray, 2))

        radius_x = size.width() / 2
        radius_y = size.height() / 2

        num_ticks = len(angles)
        step = (max_value - min_value) / (num_ticks - 1)
        value = min_value

        i = 0
        for angle in angles:
            angle_rad = math.radians(angle)

            outer_point = QtCore.QPointF(
                center.x() + (radius_x * 0.7) * math.cos(angle_rad),
                center.y() - (radius_y * 0.7) * math.sin(angle_rad)
            )

            outer_point_half = QtCore.QPointF(
                center.x() + (radius_x * 0.80) * math.cos(angle_rad),
                center.y() - (radius_y * 0.80) * math.sin(angle_rad)
            )

            outer_point_neg = QtCore.QPointF(
                center.x() + (radius_x * 0.7) * math.cos(angle_rad),
                center.y() - (radius_y * 0.7) * math.sin(angle_rad)
            )

            x = outer_point_neg.x() if value < 0 else outer_point.x()
            y = outer_point_neg.y() if value < 0 else outer_point.y()

            text_rec = QtCore.QRectF(
                x - 20,
                y - 10,
                40,
                20
            )

            if i % 2 != 0 and value >= 0:
                text_rec = QtCore.QRectF(
                    outer_point_half.x() - 20,
                    outer_point_half.y() - 10,
                    40,
                    20
                )

            painter.drawText(text_rec, QtCore.Qt.AlignCenter, f"{value:.0f}")
            value += step
            i += 1

    def draw_yaw_increments(self, painter: QPainter, center: QtCore.QSizeF, size: QtCore.QSizeF, angles: list, min_value: float, max_value: float) -> None:
        font = painter.font()
        font.setPointSize(8)
        painter.setFont(font)
        painter.setPen(QPen(QtCore.Qt.lightGray, 2))

        radius_x = size.width() / 2
        radius_y = size.height() / 2

        num_ticks = len(angles)
        step = (max_value - min_value) / (num_ticks - 1)
        value = min_value

        for angle in angles:
            angle_rad = math.radians(angle)

            outer_point = QtCore.QPointF(
                center.x() + (radius_x * 0.7) * math.cos(angle_rad),
                center.y() - (radius_y * 0.7) * math.sin(angle_rad)
            )

            outer_point_neg = QtCore.QPointF(
                center.x() + (radius_x * 0.6) * math.cos(angle_rad),
                center.y() - (radius_y * 0.6) * math.sin(angle_rad)
            )

            x = outer_point_neg.x() if value < 0 else outer_point.x()
            y = outer_point_neg.y() if value < 0 else outer_point.y()

            text_rec = QtCore.QRectF(
                x - 20,
                y - 10,
                40,
                20
            )

            painter.drawText(text_rec, QtCore.Qt.AlignCenter, f"{value:.0f}")
            value += step

    def draw_needle_right(self, painter: QPainter, center: QtCore.QSizeF, size: QtCore.QSizeF, angles: list, current_value: float, min_value: float, max_value: float) -> None:
        total_angle = max(angles) - min(angles)
        frac = (current_value - min_value) / (max_value - min_value)
        angle = min(angles) + total_angle * frac
        radius_x = size.width() / 2
        radius_y = size.height() / 2
        angle_rad = math.radians(angle)
        outer_point = QtCore.QPointF(
            center.x() + (radius_x * 0.4) * math.cos(angle_rad),
            center.y() - (radius_y * 0.4) * math.sin(angle_rad)
        )

        needle_width = 4
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QColor(200, 0, 0))

        perp_angle = angle_rad + math.pi / 2
        base_offset_x = math.cos(perp_angle) * needle_width / 2
        base_offset_y = -math.sin(perp_angle) * needle_width / 2

        triangle_points = [
            outer_point,
            QtCore.QPointF(
                center.x() + base_offset_x,
                center.y() + base_offset_y
            ),
            QtCore.QPointF(
                center.x() - base_offset_x,
                center.y() - base_offset_y
            )
        ]

        painter.drawPolygon(QPolygonF(triangle_points))
        painter.setPen(QPen(QColor(0, 51, 204), 5))
        painter.drawEllipse(center, 3, 3)

    def draw_needle_left(self, painter: QPainter, center: QtCore.QSizeF, size: QtCore.QSizeF, angles: list, current_value: float, min_value: float, max_value: float) -> None:
        total_angle = max(angles) - min(angles)
        frac = (current_value - min_value) / (max_value - min_value)
        angle = max(angles) - total_angle * frac
        radius_x = size.width() / 2
        radius_y = size.height() / 2
        angle_rad = math.radians(angle)
        outer_point = QtCore.QPointF(
            center.x() + (radius_x * 0.6) * math.cos(angle_rad),
            center.y() - (radius_y * 0.6) * math.sin(angle_rad)
        )

        needle_width = 4
        painter.setPen(QtCore.Qt.NoPen)
        painter.setBrush(QColor(200, 0, 0))

        perp_angle = angle_rad + math.pi / 2
        base_offset_x = math.cos(perp_angle) * needle_width / 2
        base_offset_y = -math.sin(perp_angle) * needle_width / 2

        triangle_points = [
            outer_point,
            QtCore.QPointF(
                center.x() + base_offset_x,
                center.y() + base_offset_y
            ),
            QtCore.QPointF(
                center.x() - base_offset_x,
                center.y() - base_offset_y
            )
        ]

        painter.drawPolygon(QPolygonF(triangle_points))
        painter.setPen(QPen(QColor(0, 51, 204), 5))
        painter.drawEllipse(center, 3, 3)
