from PyQt5.QtCore import Qt, QRectF, QTimer
from PyQt5.QtGui import QPainter, QPen, QColor, QConicalGradient
from PyQt5.QtWidgets import QWidget, QSizePolicy


class LoadingSpinner(QWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.angle = 0
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_angle)
        self.timer.start(50)

    def update_angle(self):
        self.angle = (self.angle + 12) % 360  # Increment angle by 6 degrees each update
        self.update()

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        # Calculate dimensions
        width = self.width() - 8
        height = self.height() - 8
        size = min(width, height) * 0.9

        center_x = self.width() / 2
        center_y = self.height() / 2

        rect = QRectF(center_x - size / 2, center_y - size / 2, size, size)

        # Create conical gradient that rotates with the animation
        gradient = QConicalGradient(rect.center(), self.angle - 5)
        gradient.setColorAt(0.0, QColor(65, 105, 225))
        gradient.setColorAt(0.45, QColor(51, 204, 204))
        gradient.setColorAt(0.9, QColor(255, 255, 255))

        # Configure pen for drawing
        pen = QPen(gradient, 12, Qt.SolidLine)
        painter.setPen(pen)

        # Draw 80% arc (288 degrees = 360 * 0.8)
        painter.drawArc(rect, self.angle * 16, 288 * 16)

    def showEvent(self, event):
        self.timer.start()
        super().showEvent(event)

    def hideEvent(self, event):
        self.timer.stop()
        super().hideEvent(event)
