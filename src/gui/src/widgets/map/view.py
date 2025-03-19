from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QGraphicsView, QSizePolicy, QLabel, QWidget
from .utils import MapUtils


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
        self.map_widget = self.parent()
        map_utils = MapUtils()
        self.nodes = map_utils.get_all_nodes()
        self.node_btns = [
            QtWidgets.QPushButton(
                parent=self,
                toolTip=f"Real-world position: ({node[1]:.2f}, {node[2]:.2f})",
            )
            for node in self.nodes
        ]
        for btn in self.node_btns:
            btn.hide()

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

    def show_nodes(self) -> None:
        if self.map_widget.show_nodes:
            view_width = self.width()
            view_height = self.height()
            for node, btn in zip(self.nodes, self.node_btns):
                x_px = node[1] / self.map_widget.real_x_per_pixel
                y_px = node[2] / self.map_widget.real_y_per_pixel
                viewport_pos = self.mapFromScene(x_px, y_px)
                btn_x = viewport_pos.x() - btn.width() // 2
                btn_y = viewport_pos.y() - btn.height() // 2
                btn.move(btn_x, btn_y)
                is_visible = (
                    btn_x + btn.width() > 0 and btn_x < view_width and btn_y + btn.height() > 0 and btn_y < view_height
                )
                btn.setVisible(is_visible)
        else:
            for btn in self.node_btns:
                btn.hide()

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
