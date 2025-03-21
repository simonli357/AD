from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QGraphicsView, QSizePolicy, QLabel, QWidget
from .utils import MapUtils
from ..enums import MapData

import numpy as np


class NodeButton(QtWidgets.QPushButton):
    def __init__(self, node_data, parent=None):
        super().__init__(parent)
        self.node_data = node_data
        self.is_clicked = False
        self.is_start = False
        self.setToolTip(f"  {node_data[0]}: ({node_data[1]:.2f}, {node_data[2]:.2f})")

    def paintEvent(self, event):
        """Custom painting with proper visible area"""
        super().paintEvent(event)
        painter = QtGui.QPainter(self)
        painter.setRenderHint(QtGui.QPainter.Antialiasing)
        padding = int(self.width() * 0.4)
        visible_rect = self.rect().adjusted(
            padding,
            padding,
            -padding,
            -padding
        )
        painter.setBrush(QtGui.QColor(255, 255, 255, 128))
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 0, 200), 2))
        painter.drawEllipse(visible_rect)


class GraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
        )
        self._pan_start = None
        self._is_dragging = False
        self.map_widget = self.parent()
        map_utils = MapUtils()
        self.nodes = map_utils.get_all_nodes()
        self.destinations = map_utils.get_destination_nodes()
        self.node_btns = [
            NodeButton(node_data=node, parent=self)
            for node in self.nodes
        ]
        for btn in self.node_btns:
            btn.clicked.connect(lambda _, b=btn: self.on_node_click(b))
            btn.hide()
        self.path = []
        self.visited = {}
        self.setStyleSheet("""
            QPushButton {
                border: none;
                background-color: rgba(0,0,0,0);
            }
            QToolTip {
                background-color: black;
                color: #00ff00;
                border: none;
                font-size: 14px;
                padding: 5px;
            }
        """)
        self.setup_ui()
        self.setup_cursor()

    def setup_ui(self):
        self.total_dist_label = QLabel('󰣰 Distance: --:--')
        self.total_dist_label.setStyleSheet("""
            border: none;
            padding: 5px;
            background-color: transparent;
            color: yellow;
            font-size: 20px;
        """)
        self.dist_traveled_label = QLabel('  Traveled: --:--')
        self.dist_traveled_label.setStyleSheet("""
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
        self.wrapper.addWidget(self.dist_traveled_label)
        self.wrapper.addWidget(self.dest_reached_label)
        self.overlay_widget.move(
            self.width() - self.overlay_widget.width() - 5,
            5
        )

    def setup_cursor(self) -> None:
        crosshair_pixmap = QtGui.QPixmap(16, 16)
        crosshair_pixmap.fill(QtCore.Qt.transparent)
        painter = QtGui.QPainter(crosshair_pixmap)
        painter.setPen(QtGui.QPen(QtGui.QColor(0, 255, 0), 1))
        painter.drawLine(8, 0, 8, 16)
        painter.drawLine(0, 8, 16, 8)
        painter.end()
        self.setCursor(QtGui.QCursor(crosshair_pixmap, 8, 8))
        self.viewport().setCursor(QtGui.QCursor(crosshair_pixmap, 8, 8))

    def update_btn_style(self, button, size):
        """Update button color based on boolean state"""
        color = "#ff00ff" if button.is_clicked else "rgba(0,0,0,0)"
        color = "#ffff00" if button.is_start else color
        button.setStyleSheet(f"""
            QPushButton {{
                border-radius: {size // 2}px;
                background-color: {color};
            }}
            QPushButton:hover {{
                background-color: rgba(0, 255, 0, 0.5);
            }}
        """)

    def show_nodes(self) -> None:
        if self.map_widget.show_nodes:
            mouse_pos = self.mapFromGlobal(QtGui.QCursor.pos())
            mouse_radius = 50
            mouse_radius_sq = mouse_radius ** 2
            view_width = self.width()
            view_height = self.height()
            for node, btn in zip(self.nodes, self.node_btns):
                x_px = node[1] / self.map_widget.real_x_per_pixel
                y_px = node[2] / self.map_widget.real_y_per_pixel
                viewport_pos = self.mapFromScene(x_px, y_px)
                dx = viewport_pos.x() - mouse_pos.x()
                dy = viewport_pos.y() - mouse_pos.y()
                distance_sq = dx * dx + dy * dy
                if distance_sq <= mouse_radius_sq or btn.is_clicked:
                    size = 14 * self.map_widget.current_zoom
                    btn.setFixedSize(int(size), int(size))
                    self.update_btn_style(btn, size)
                    btn_x = viewport_pos.x() - size // 2
                    btn_y = viewport_pos.y() - size // 2
                    is_visible = (btn_x + size > 0 and btn_x < view_width and btn_y + size > 0 and btn_y < view_height)
                    if is_visible:
                        btn.move(int(btn_x), int(btn_y))
                    btn.setVisible(is_visible)
                else:
                    btn.hide()
        else:
            for btn in self.node_btns:
                btn.hide()

    def is_button_start(self, button):
        for btn in self.node_btns:
            if btn.node_data[0] == button.node_data[0]:
                continue
            if btn.is_clicked:
                return False
        return True

    def on_node_click(self, button):
        button.is_clicked = not button.is_clicked
        if button.is_clicked:
            button.is_start = self.is_button_start(button)
            if button.is_start:
                self.path.insert(0, button)
            else:
                self.path.append(button)
        else:
            button.is_start = False
            self.path.remove(button)
        self.update_btn_style(button, button.size().width())

    def get_path(self):
        path = []
        for btn in self.path[1:]:
            path.append((btn.node_data[1], MapData.REAL_WORLD_HEIGHT.value - btn.node_data[2]))
        return path

    def get_start(self):
        x_px = self.path[0].node_data[1]
        y_px = MapData.REAL_WORLD_HEIGHT.value - self.path[0].node_data[2]
        return (x_px, y_px)

    def clear_path(self):
        self.path.clear()
        for btn in self.node_btns:
            btn.is_start = False
            btn.is_clicked = False

    def calculate_total_path_distance(self):
        if self.map_widget.state_refs_np.shape[1] < 2:
            return 0.0
        x_coords = self.map_widget.state_refs_np[0, :]
        y_coords = self.map_widget.state_refs_np[1, :]
        dx = x_coords[1:] - x_coords[:-1]
        dy = y_coords[1:] - y_coords[:-1]
        distances = np.sqrt(dx**2 + dy**2)
        return np.sum(distances)

    def is_near(self, x1: float, y1: float, x2: float, y2: float, rad1: float, rad2: float):
        return (x2 - x1)**2 + (y2 - y1)**2 <= (rad1 + rad2)**2

    def update_visited_destinations(self, car_x: float, car_y: float):
        for id, x, y in self.destinations:
            if self.is_near(car_x, car_y, x, y, 0.2, 0.2):
                self.visited.append(id)
                self.set_dest_visited_num(len(self.visited))
                break

    def set_total_path_distance(self):
        dist = self.calculate_total_path_distance()
        self.total_dist_label.setText(f'󰣰 Distance: {dist:.2f}')

    def set_distance_traveled(self, dist: float) -> None:
        self.dist_traveled_label.setText(f'  Traveled: {dist:.2f}')

    def set_dest_visited_num(self, dest_visited: int) -> None:
        self.dest_reached_label.setText(f'󰪥 Reached: {dest_visited:.0f}')

    #################
    # Events
    #################

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

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._pan_start = event.pos()
            event.accept()
        else:
            super().mousePressEvent(event)

    def mouseMoveEvent(self, event):
        if self._pan_start is not None:
            delta = event.pos() - self._pan_start
            self.horizontalScrollBar().setValue(
                self.horizontalScrollBar().value() - delta.x()
            )
            self.verticalScrollBar().setValue(
                self.verticalScrollBar().value() - delta.y()
            )
            self._pan_start = event.pos()
            event.accept()
        super().mouseMoveEvent(event)

    def mouseReleaseEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self._pan_start = None
            event.accept()
        else:
            super().mouseReleaseEvent(event)
