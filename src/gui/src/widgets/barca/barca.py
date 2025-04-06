from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
from ..enums import BarcaMapData
from ..opengl.shader import ShaderRenderer
from ..opengl.waypoints import WaypointsRenderer

import glm
import numpy as np


class BarcaWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.stop_drawing = False
        self.main_window = self.parent()
        self.car_widget = self.main_window.car_widget
        self.last_mouse_pos = None
        self.current_mouse_pos = None
        self.show_mouse = True

        self.view_center = glm.vec2(4.0, -4.0)
        self.view_zoom = 17.0
        self.drag_start = None
        self.base_view_center = glm.vec2(self.view_center)

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
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glDisable(gl.GL_BLEND)          # Disable unless transparency needed
        gl.glDisable(gl.GL_LINE_SMOOTH)    # Avoid anti-aliasing overhead
        gl.glDisable(gl.GL_POLYGON_SMOOTH)
        gl.glDisable(gl.GL_MULTISAMPLE)    # Disable MSAA if not used
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)  # Fastest mode
        gl.glShadeModel(gl.GL_FLAT)        # Faster than GL_SMOOTH if applicable

        self.waypoints_renderer = WaypointsRenderer(track='barca')
        self.shader_renderer = ShaderRenderer()

    def paintGL(self):
        if self.stop_drawing:
            return
        if self.view_zoom == 17.0:
            self.view_center = glm.vec2(4, -4)

        self.update_mouse_pos()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        zoom_factor = 1.0 / self.view_zoom
        half_width = zoom_factor * self.width() / 2
        half_height = zoom_factor * self.height() / 2

        self.proj_mat = glm.ortho(
            -half_width, half_width,
            -half_height, half_height,
            0.1, 100.0
        )

        self.view_mat = glm.lookAt(
            glm.vec3(self.view_center.x, self.view_center.y, 1.0),
            glm.vec3(self.view_center.x, self.view_center.y, 0.0),
            glm.vec3(0.0, 1.0, 0.0)
        )

        # Draw objects
        self.waypoints_renderer.draw(self.proj_mat, self.view_mat)
        self.shader_renderer.draw_barca_track(-2, -4, 0, 0, (1.0, 1.0), (0.0, 1.0, 0.0, 1.0), self.view_mat, self.proj_mat)
        self.shader_renderer.draw_car(self.car_widget.x_pos, self.car_widget.y_pos, self.car_widget.yaw + np.radians(-90), 0.01, self.view_mat, self.proj_mat)
        self.draw_path_nodes(self.main_window.map_widget.waypoints)

    def draw_path_nodes(self, waypoints):
        if waypoints is None or len(waypoints) < 2:
            return
        x1, y1 = waypoints[0], waypoints[1]
        angle = 0
        for i in range(0, len(waypoints) - 1, 4):
            if i + 3 > len(waypoints):
                self.shader_renderer.draw_triangle(x1, y1, 0, angle, (0.5, 0.5), (1.0, 1.0, 0.0, 1.0), self.view_mat, self.proj_mat)
            else:
                x2, y2 = waypoints[i + 2], waypoints[i + 3]
                dx = x2 - x1
                dy = y2 - y1
                angle = np.arctan2(dy, dx + (1e-5)) - np.pi / 2
                self.shader_renderer.draw_triangle(x1, y1, 0, angle, (0.5, 0.5), (1.0, 1.0, 0.0, 1.0), self.view_mat, self.proj_mat)
                x1, y1 = x2, y2

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

            x_world, y_world = self.get_real_world_coords(x_scene, y_scene)
            # Update label text and position
            self.cursor_coords_label.setText(f"  ({x_world:.2f}, {y_world:.2f}) ")
            self.cursor_coords_label.move(
                int(x_scene - self.cursor_coords_label.width() / 2),
                int(y_scene - 60))

    def get_real_world_coords(self, x_scene, y_scene):
        widget_width = self.width()
        widget_height = self.height()
        if widget_height == 0 or widget_width == 0:
            return (0.0, 0.0)

        # Convert screen coordinates to NDC (Normalized Device Coordinates)
        ndc_x = (2.0 * x_scene / widget_width) - 1.0
        ndc_y = 1.0 - (2.0 * y_scene / widget_height)

        # Compute half width and height in view space after zoom
        hw = (widget_width / self.view_zoom) / 2.0
        hh = (widget_height / self.view_zoom) / 2.0

        # Apply inverse projection to get view space coordinates
        x_view = ndc_x * hw
        y_view = ndc_y * hh

        # Apply inverse view matrix to get OpenGL world coordinates
        world_x = x_view + self.view_center.x
        world_y = y_view + self.view_center.y

        # Convert OpenGL coordinates to real-world system
        real_world_x = (world_x + (widget_width / 2)) * (BarcaMapData.REAL_WORLD_WIDTH.value / widget_width)
        real_world_y = (world_y + (widget_height / 2)) * (BarcaMapData.REAL_WORLD_HEIGHT.value / widget_height)

        return (real_world_x - 35) * 100, (real_world_y - 12.50) * 100

    def update_waypoints(self):
        if hasattr(self, 'proj_mat') and hasattr(self, 'view_mat'):
            self.waypoints_renderer.update_waypoints(self.main_window.map_widget.state_refs_np, self.main_window.map_widget.attributes_np, self.width(), self.height())

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
        self.update_waypoints()

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.drag_start = event.pos()
            self.base_view_center = glm.vec2(self.view_center)
            self.last_mouse_pos = event.pos()
        if event.button() == QtCore.Qt.RightButton:
            self.cursor_coords.clear()

    def mouseMoveEvent(self, event):
        self.current_mouse_pos = event.pos()
        if event.buttons() & QtCore.Qt.LeftButton and self.drag_start is not None:
            # Calculate delta movement
            delta = event.pos() - self.last_mouse_pos
            self.last_mouse_pos = event.pos()

            # Convert pixel delta to world coordinates
            zoom_factor = 1.0 / self.view_zoom
            half_width = self.width() * zoom_factor / 2
            half_height = self.height() * zoom_factor / 2

            # Calculate world units per pixel
            world_per_pixel_x = (2 * half_width) / self.width()
            world_per_pixel_y = (2 * half_height) / self.height()

            # Update view center
            self.view_center.x -= delta.x() * world_per_pixel_x
            self.view_center.y += delta.y() * world_per_pixel_y

    def wheelEvent(self, event):
        # Get mouse position in normalized device coordinates
        mouse_pos = event.pos()
        mouse_x = 2.0 * mouse_pos.x() / self.width() - 1.0
        mouse_y = 1.0 - 2.0 * mouse_pos.y() / self.height()

        # Store pre-zoom values
        old_zoom = self.view_zoom
        zoom_factor = 1.15 if event.angleDelta().y() > 0 else 0.85
        self.view_zoom *= zoom_factor

        # Keep zoom within bounds
        self.view_zoom = max(17.0, min(80.0, self.view_zoom))

        # Calculate new center to maintain mouse position
        zoom_ratio = old_zoom / self.view_zoom
        self.view_center += glm.vec2(
            mouse_x * (self.width() / 2) * (1 - zoom_ratio) / old_zoom,
            mouse_y * (self.height() / 2) * (1 - zoom_ratio) / old_zoom
        )
