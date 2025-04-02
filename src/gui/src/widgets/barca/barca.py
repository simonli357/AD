from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
from .renderer import draw_track, draw_grid
from .vbos import grid_vbo
from .waypoints import WaypointsRenderer
from ..enums import BarcaMapData
from ..opengl.loaders import load_obj
from ..opengl.renderer import GlobalRenderer

import os


class BarcaWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.stop_drawing = False
        self.main_window = self.parent()
        self.car_widget = self.main_window.car_widget
        self.pan_x = 0
        self.pan_y = 0
        self.max_zoom = 40
        self.zoom_level = self.max_zoom
        self.last_mouse_pos = None
        self.current_mouse_pos = None
        self.show_mouse = True

        self.waypoints_renderer = WaypointsRenderer()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.track_model_path = os.path.join(current_dir, 'assets', 'track.obj')
        self.car_model_path = os.path.join(current_dir, 'assets', 'car.obj')

        self.setup_ui()

    def setup_ui(self):
        self.cursor_coords_label = QtWidgets.QLabel(self)
        self.cursor_coords_label.setStyleSheet("""
            border: none;
            background-color: rgba(0, 0, 0, 0.7);
            color: #00ff00;
            font-size: 16px;
        """)
        self.cursor_coords_label.hide()
        self.cursor_coords_label.setAlignment(QtCore.Qt.AlignCenter)

    def render_widget(self) -> None:
        self.update()

    def initializeGL(self):
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)
        gl.glEnable(gl.GL_CULL_FACE)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        self.renderer = GlobalRenderer()
        self.grid_model = grid_vbo()
        self.track_model = load_obj(self.track_model_path)
        self.car_model = load_obj(self.car_model_path)

    def paintGL(self):
        if self.stop_drawing:
            return

        self.update_mouse_pos()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glPushAttrib(gl.GL_ALL_ATTRIB_BITS)

        # Set up projection matrix
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()

        aspect = self.width() / self.height() if self.height() != 0 else 1.0
        half_zoom = self.zoom_level * 0.5
        left = self.pan_x - half_zoom * aspect
        right = self.pan_x + half_zoom * aspect
        bottom = self.pan_y - half_zoom
        top = self.pan_y + half_zoom
        gl.glOrtho(left, right, bottom, top, -100, 100)

        # Global Transforms
        gl.glTranslatef(-5.55, 1.5, 0)
        gl.glScalef(0.99, 0.99, 0.99)

        draw_track(self.renderer.barca_track)
        # Counter map offset
        gl.glTranslatef(BarcaMapData.MAP_CENTER_X.value, -BarcaMapData.MAP_CENTER_Y.value, 0)
        draw_grid(self.grid_model)
        gl.glTranslatef(-2, 6, 0)

        # Draw objects
        self.waypoints_renderer.draw()
        self.renderer.draw_car(self.car_widget.x_pos, self.car_widget.y_pos, -self.car_widget.yaw + 90, 0.01, (1.0, 0.0, 0.0, 1.0))

        gl.glPopAttrib()

    def render_text(self, text, size, color: (int, int, int, int), x, y) -> None:
        painter = QPainter(self)
        painter.setRenderHints(
            QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform
        )

        # Get current OpenGL color
        gl_color = gl.glGetDoublev(gl.GL_CURRENT_COLOR)
        text_color = QColor(
            int(gl_color[0] * color[0]),
            int(gl_color[1] * color[1]),
            int(gl_color[2] * color[2]),
            int(gl_color[3] * color[3])
        )

        # Set up font
        font = QFont("Arial")
        font.setBold(True)
        font.setStyleStrategy(QFont.PreferAntialias)

        # Account for high-DPI scaling
        scale_factor = self.devicePixelRatio()
        painter.scale(1 / scale_factor, 1 / scale_factor)
        font.setPixelSize(size * scale_factor)

        painter.setPen(text_color)
        painter.setFont(font)
        painter.drawText(int(x * scale_factor),
                         int(y * scale_factor),
                         text)
        painter.end()

    def cleanup_gl_resources(self):
        self.stop_drawing = True
        try:
            gl.glFlush()
        except Exception:
            pass

    def update_mouse_pos(self):
        if self.show_mouse:
            self.cursor_coords_label.show()
        else:
            self.cursor_coords_label.hide()
            return

        if self.current_mouse_pos is not None:
            widget_width = self.width()
            widget_height = self.height()
            if widget_height == 0 or widget_width == 0:
                return

            # Get mouse position in widget coordinates
            x_scene = self.current_mouse_pos.x()
            y_scene = self.current_mouse_pos.y()

            # Convert to normalized device coordinates [-1, 1]
            x_ndc = 2 * (x_scene / widget_width) - 1
            y_ndc = (1 - 2 * (y_scene / widget_height))

            # Calculate orthographic projection bounds
            aspect = widget_width / widget_height
            half_zoom = self.zoom_level * 0.5
            left = self.pan_x - half_zoom * aspect
            right = self.pan_x + half_zoom * aspect
            bottom = self.pan_y - half_zoom
            top = self.pan_y + half_zoom

            # Convert to world coordinates
            x_world = left + (x_ndc + 1) * (right - left) / 2
            y_world = bottom + (y_ndc + 1) * (top - bottom) / 2

            # Apply inverse of modelview transformations
            # Reverse scaling (0.99) and translation (-5.55, 1.5)
            adjusted_x = (x_world / 0.99) + 3.60
            adjusted_y = (y_world / 0.99) + 4.55

            # Apply additional map offsets
            final_x = adjusted_x - BarcaMapData.MAP_CENTER_X.value + 2
            final_y = adjusted_y + BarcaMapData.MAP_CENTER_Y.value - 6

            self.cursor_coords_label.setText(f"  ({final_x:.2f}, {final_y:.2f}) ")
            self.cursor_coords_label.move(
                int(x_scene - self.cursor_coords_label.width() / 2),
                int(y_scene - 60))

    def __del__(self):
        self.cleanup_gl_resources()

    def deleteLater(self):
        self.cleanup_gl_resources()
        super().deleteLater()

    ###############
    # Events
    ###############

    def resizeGL(self, w, h):
        gl.glViewport(0, 0, w, h)
        self.update_mouse_pos()

    def mousePressEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        self.current_mouse_pos = event.pos()
        if event.buttons() == QtCore.Qt.LeftButton and self.last_mouse_pos is not None:
            # Prevent panning when at initial zoom
            if self.zoom_level >= self.max_zoom:
                return

            dx = event.pos().x() - self.last_mouse_pos.x()
            dy = event.pos().y() - self.last_mouse_pos.y()
            self.last_mouse_pos = event.pos()

            widget_height = self.height()
            if widget_height == 0:
                widget_height = 1

            aspect = self.width() / widget_height
            scale = self.zoom_level / widget_height

            # Calculate proposed pan changes
            new_pan_x = self.pan_x - dx * scale
            new_pan_y = self.pan_y + dy * scale

            # Calculate content boundaries based on initial zoom
            half_span_x = (self.max_zoom - self.zoom_level) * aspect / 2
            half_span_y = (self.max_zoom - self.zoom_level) / 2

            # Clamp pan values to content boundaries
            self.pan_x = max(-half_span_x, min(half_span_x, new_pan_x))
            self.pan_y = max(-half_span_y, min(half_span_y, new_pan_y))

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta != 0:
            new_zoom = self.zoom_level - delta * 0.015
            new_zoom = max(8, min(self.max_zoom, new_zoom))
            if new_zoom != self.zoom_level:
                self.zoom_level = new_zoom
                # Reset pan when returning to initial zoom
                if self.zoom_level == self.max_zoom:
                    self.pan_x = 0
                    self.pan_y = 0
