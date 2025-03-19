from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QGraphicsView, QSizePolicy, QLabel, QWidget


class GraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
        )
        self.zoom_factor = 1.10
        self.min_zoom = 1.0
        self.max_zoom = 10.0

        self.total_dist_label = QLabel('󰣰 Distance: --:--')
        self.total_dist_label.setStyleSheet("""
            border: none;
            padding: 5px;
            background-color: transparent;
            color: yellow;
            font-size: 20px;
        """)

        self.current_dist_label = QLabel('  Traveled: --:--')
        self.current_dist_label.setStyleSheet("""
            border: none;
            padding: 5px;
            background-color: transparent;
            color: yellow;
            font-size: 20px;
        """)

        self.dest_reached_label = QLabel('󰪥 Reached: --:--')
        self.dest_reached_label.setStyleSheet("""
            border: none;
            padding: 5px;
            background-color: transparent;
            color: yellow;
            font-size: 20px;
        """)

        self.setup_ui()

    def setup_ui(self):
        self.overlay_widget = QWidget(self)
        self.overlay_widget.setStyleSheet("""
            background: rgba(40, 40, 40, 0.8);
            border: none;
            border-radius: 8px;
        """)
        self.overlay_widget.setFixedSize(
            int(self.width() * 0.25),
            int(self.height() * 0.15)
        )
        self.wrapper = QtWidgets.QVBoxLayout(self.overlay_widget)
        self.wrapper.setAlignment(QtCore.Qt.AlignVCenter | QtCore.Qt.AlignLeft)
        self.wrapper.setContentsMargins(10, 10, 10, 10)
        self.wrapper.addWidget(self.total_dist_label)
        self.wrapper.addWidget(self.current_dist_label)
        self.wrapper.addWidget(self.dest_reached_label)
        self.overlay_widget.move(
            self.width() - self.overlay_widget.width() - 5,
            5
        )

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.overlay_widget.setFixedSize(
            256,
            144
        ),
        self.overlay_widget.move(
            event.size().width() - self.overlay_widget.width() - 5,
            5
        ),
        event.accept()

    def wheelEvent(self, event):
        current_scale = self.transform().m11()
        if event.angleDelta().y() > 0:
            new_scale = current_scale * self.zoom_factor
        else:
            new_scale = current_scale / self.zoom_factor
        if new_scale < self.min_zoom or new_scale > self.max_zoom:
            event.accept()
            return
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        if event.angleDelta().y() > 0:
            self.scale(self.zoom_factor, self.zoom_factor)
        else:
            self.scale(1 / self.zoom_factor, 1 / self.zoom_factor)
        event.accept()
