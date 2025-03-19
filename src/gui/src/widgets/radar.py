from PyQt5.QtGui import QPainter, QPen, QColor, QImage, QPixmap
from PyQt5 import QtCore, QtWidgets

import cv2
import os


class RadarWidget(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.main_window = self.parent()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.car_icon_path = os.path.join(current_dir, 'assets', 'car_top.png')

        self.obj_dict = self.main_window.map_widget.object_dict
        self.sign_images = self.main_window.map_widget.sign_images

        self.x_min, self.x_max = -1.25, 1.25
        self.y_min, self.y_max = -1.25, 1.25

    def render_widget(self) -> None:
        self.update()

    def paintEvent(self, event):
        with QPainter(self) as painter:
            painter.setRenderHint(QPainter.Antialiasing)
            dimensions = self.get_dimensions()
            h1, h2, w1, w2, mid_h, mid_w = dimensions
            try:
                pixels_per_meter_x = (w2 - w1) / (self.x_max - self.x_min)
                pixels_per_meter_y = (h2 - h1) / (self.y_max - self.y_min)
            except ZeroDivisionError:
                return
            self.draw_grid(painter, h1, h2, w1, w2, mid_h, mid_w, pixels_per_meter_x, pixels_per_meter_y)
            self.draw_axis(painter, h1, h2, w1, w2, mid_h, mid_w)
            self.draw_car(painter, mid_w + 15, mid_h + 60, 50)
            self.draw_detected_object(painter, mid_h, mid_w, h2, pixels_per_meter_x, pixels_per_meter_y)

    def draw_sign(self, painter: QPainter, x, y, size, sign_type):
        img = self.sign_images[int(sign_type)]
        if img is not None:
            size = max(5, size)
            img = cv2.resize(img, (size, size))
            rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
            height, width, _ = rgb_img.shape
            bytes_per_line = 3 * width
            qimg = QImage(rgb_img.data, width, height, bytes_per_line, QImage.Format_RGB888)
            pixmap = QPixmap.fromImage(qimg)
            image_rect = QtCore.QRectF(x - size + 10, y - size - 15, size, size)
            painter.drawPixmap(image_rect.topLeft(), pixmap)

    def draw_car(self, painter: QPainter, x, y, size):
        img = cv2.imread(self.car_icon_path, cv2.IMREAD_UNCHANGED)  # Loads BGRA (if PNG with alpha)
        if img is not None:
            size = max(5, size)
            img = cv2.resize(img, (size, size))
            if img.shape[2] == 4:  # Check if alpha channel exists
                rgb_img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)
                qt_format = QImage.Format_RGBA8888
                bytes_per_line = 4 * img.shape[1]  # 4 channels (RGBA)
            else:
                rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                qt_format = QImage.Format_RGB888
                bytes_per_line = 3 * img.shape[1]
            height, width, _ = rgb_img.shape
            qimg = QImage(
                rgb_img.data,
                width,
                height,
                bytes_per_line,
                qt_format
            )
            pixmap = QPixmap.fromImage(qimg)
            image_rect = QtCore.QRectF(x - size + 10, y - size - 15, size, size)
            painter.drawPixmap(image_rect.topLeft(), pixmap)

    def get_dimensions(self) -> tuple:
        # Calculate axis positions
        h1 = 0.0
        h2 = self.height()
        w1 = 0.0
        w2 = self.width()
        mid_h = self.height() / 1.25
        mid_w = self.width() / 2
        return (h1, h2, w1, w2, mid_h, mid_w)

    def draw_axis(self, painter: QPainter, h1, h2, w1, w2, mid_h, mid_w) -> None:
        # Draw vertical axis (Y-axis)
        painter.setPen(QPen(QColor(0, 255, 0), 1))
        painter.drawLine(QtCore.QPointF(mid_w, h1), QtCore.QPointF(mid_w, h2))

        # Draw horizontal axis (X-axis)
        painter.setPen(QPen(QColor(255, 0, 0), 1))
        painter.drawLine(QtCore.QPointF(w1, mid_h), QtCore.QPointF(w2, mid_h))

    def draw_grid(self, painter: QPainter, h1: float, h2: float, w1: float, w2: float, mid_h: float, mid_w: float, pixels_per_meter_x: float, pixels_per_meter_y: float, divisions=2) -> None:
        painter.setPen(QPen(QColor(100, 100, 100, 180), 1))
        x = mid_w + pixels_per_meter_x / divisions
        while x <= w2:
            if not QtCore.qFuzzyCompare(x, mid_w):
                painter.drawLine(QtCore.QPointF(x, h1), QtCore.QPointF(x, h2))
            x += pixels_per_meter_x / divisions

        x = mid_w - pixels_per_meter_x / divisions
        while x >= w1:
            if not QtCore.qFuzzyCompare(x, mid_w):
                painter.drawLine(QtCore.QPointF(x, h1), QtCore.QPointF(x, h2))
            x -= pixels_per_meter_x / divisions

        y = h2 - pixels_per_meter_y / divisions
        while y > h1:
            if not QtCore.qFuzzyCompare(y, mid_h):
                painter.drawLine(QtCore.QPointF(w1, y), QtCore.QPointF(w2, y))
            y -= pixels_per_meter_y / divisions

    def draw_detected_object(self, painter: QPainter, mid_h: float, mid_w: float, h2: float, pixels_per_meter_x: float, pixels_per_meter_y: float):
        obj = self.main_window.cam_widget.detected_objects
        # Convert real-world coordinates to widget coordinates
        for i in range(0, self.main_window.cam_widget.numObj):
            if obj is None:
                return

            obj_type = obj[10 * i + 6]

            widget_x = mid_w - obj[i * 10 + 8] * pixels_per_meter_x
            widget_y = mid_h - obj[i * 10 + 7] * pixels_per_meter_y

            if self.obj_dict[obj_type] == 'Car':
                painter.setPen(QPen(QColor('#009999'), 5))
            if self.obj_dict[obj_type] == 'No Entry':
                painter.setPen(QPen(QColor('#ff6600'), 5))
            if self.obj_dict[obj_type] == 'Stopsign':
                painter.setPen(QPen(QColor('#ff0000'), 5))
            if self.obj_dict[obj_type] == 'Oneway':
                painter.setPen(QPen(QColor('#ffffff'), 5))
            if self.obj_dict[obj_type] == 'Highway Entrance':
                painter.setPen(QPen(QColor('#004d00'), 5))
            if self.obj_dict[obj_type] == 'Roundabout':
                painter.setPen(QPen(QColor('#0000ff'), 5))
            if self.obj_dict[obj_type] == 'Parking':
                painter.setPen(QPen(QColor('#00ff00'), 5))
            if self.obj_dict[obj_type] == 'Crosswalk':
                painter.setPen(QPen(QColor('#ffff00'), 5))
            if self.obj_dict[obj_type] == 'Highway Exit':
                painter.setPen(QPen(QColor('#004d00'), 5))
            if self.obj_dict[obj_type] == 'Priority':
                painter.setPen(QPen(QColor('#0066ff'), 5))
            if self.obj_dict[obj_type] == 'Green Light':
                painter.setPen(QPen(QColor('#00ff00'), 5))
            if self.obj_dict[obj_type] == 'Yellow Light':
                painter.setPen(QPen(QColor('#ffff00'), 5))
            if self.obj_dict[obj_type] == 'Red Light':
                painter.setPen(QPen(QColor('#ff0000'), 5))
            if self.obj_dict[obj_type] == 'Pedestrian':
                painter.setPen(QPen(QColor('#ff3399'), 5))

            self.draw_sign(painter, widget_x, widget_y, 25, obj_type)

            painter.drawEllipse(QtCore.QPointF(widget_x, widget_y), 2, 2)

            text_rec = QtCore.QRectF(
                widget_x,
                widget_y - 20,
                80,
                40
            )

            painter.setPen(QPen(QtCore.Qt.yellow, 2))
            painter.drawText(text_rec, QtCore.Qt.AlignCenter, f"x: {-obj[i * 10 + 8]:.2f} \n y: {obj[i * 10 + 7]:.2f}")
