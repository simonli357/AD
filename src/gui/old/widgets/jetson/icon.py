from PyQt5.QtGui import QPainter, QImage, QPixmap
from PyQt5 import QtWidgets

import os
import cv2


class JetsonIcon(QtWidgets.QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.jetson_icon_path = os.path.join(current_dir, 'assets', 'jetson.png')

    def paintEvent(self, event):
        with QPainter(self) as painter:
            self.draw_jetson(painter, 150)

    def draw_jetson(self, painter: QPainter, size):
        img = cv2.imread(self.jetson_icon_path, cv2.IMREAD_UNCHANGED)
        if img is not None:
            # Get widget dimensions
            widget_width = self.width()
            widget_height = self.height()

            # Calculate image dimensions (preserve aspect ratio)
            img_height, img_width = img.shape[:2]
            aspect_ratio = img_width / img_height

            # Calculate target size while preserving aspect ratio
            target_width = min(size, widget_width)
            target_height = int(target_width / aspect_ratio)

            # Resize image
            img = cv2.resize(img, (target_width, target_height))

            # Convert color space
            if img.shape[2] == 4:
                rgb_img = cv2.cvtColor(img, cv2.COLOR_BGRA2RGBA)
                qt_format = QImage.Format_RGBA8888
                bytes_per_line = 4 * target_width
            else:
                rgb_img = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
                qt_format = QImage.Format_RGB888
                bytes_per_line = 3 * target_width

            # Create QImage and QPixmap
            qimg = QImage(
                rgb_img.data,
                target_width,
                target_height,
                bytes_per_line,
                qt_format
            )
            pixmap = QPixmap.fromImage(qimg)

            # Calculate centered position
            x = (widget_width - target_width) // 2
            y = (widget_height - target_height) // 2

            # Draw centered pixmap
            painter.drawPixmap(x, y, pixmap)
