from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
from OpenGL import GLU as glu
from ..enums import MapData
from .opengl.vbos import track_vbo
from ..opengl.renderer import GlobalRenderer

import os
import numpy as np


class CarWidget(QtWidgets.QOpenGLWidget):
    update_map_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.stop_drawing = False
        self.main_window = self.parent()

        self.steer = 0
        self.yaw = 0
        self.x_pos = 11.8
        self.y_pos = MapData.REAL_WORLD_HEIGHT.value - 2.05
        self.z_pos = 0

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.track_model_path = os.path.join(current_dir, 'assets', 'track.png')
        self.car_model_path = os.path.join(current_dir, 'assets', 'car.obj')

    def set_steer(self, steer: float) -> None:
        self.steer = steer

    def set_car_data(self, yaw: float, x: float, y: float, z: float) -> None:
        if self.main_window.buttons_widget.started:
            dx = x - self.x_pos
            dy = y - self.y_pos
            displacement = np.sqrt(dx**2 + dy**2)
            self.main_window.map_widget.run_statistics.dist_traveled += displacement
            self.main_window.map_widget.run_statistics.set_distance_traveled()
            self.main_window.map_widget.run_statistics.update_visited_destinations(x, y)
        self.yaw = yaw
        self.x_pos = x
        self.y_pos = y
        self.z_pos = z

    def render_widget(self) -> None:
        self.update()

    def initializeGL(self):
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)
        gl.glEnable(gl.GL_CULL_FACE)
        gl.glCullFace(gl.GL_BACK)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glDisable(gl.GL_BLEND)          # Disable unless transparency needed
        gl.glDisable(gl.GL_LINE_SMOOTH)    # Avoid anti-aliasing overhead
        gl.glDisable(gl.GL_POLYGON_SMOOTH)
        gl.glDisable(gl.GL_MULTISAMPLE)    # Disable MSAA if not used
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)  # Fastest mode
        gl.glShadeModel(gl.GL_FLAT)        # Faster than GL_SMOOTH if applicable

        self.renderer = GlobalRenderer()
        self.track_vbo = track_vbo(self.width(), self.height())

    def paintGL(self):
        if self.stop_drawing:
            return

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # Set up projection matrix
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = self.width() / self.height() if self.height() != 0 else 1.0
        glu.gluPerspective(45, aspect, 0.1, 100)

        # Set up view matrix
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

        x = self.x_pos / MapData.REAL_WORLD_WIDTH.value * self.width()
        y = (MapData.REAL_WORLD_HEIGHT.value - self.y_pos) / MapData.REAL_WORLD_HEIGHT.value * self.height()

        # Camera
        camera_distance = 16.0
        camera_height = camera_distance / 3 * 2
        cam_x = x - camera_distance * np.cos(np.radians(self.yaw))
        cam_y = y - camera_distance * np.sin(np.radians(self.yaw))
        cam_z = camera_height

        glu.gluLookAt(
            cam_x, cam_y, cam_z,
            x, y, 0,
            0, 0, 1
        )

        # Draw track
        gl.glPushMatrix()
        gl.glTranslatef(0, 0, -1.1)
        self.renderer.draw_2D_texture(self.renderer.bfmc_track_texture, self.track_vbo)
        gl.glPopMatrix()

        # Draw car
        self.renderer.draw_car(x, y, self.yaw, 0.2, (1.0, 0.0, 0.0, 1.0))

    def render_text(self, text, size, color: (int, int, int, int), x, y) -> None:
        painter = QPainter(self)
        painter.setRenderHints(
            QPainter.Antialiasing | QPainter.TextAntialiasing | QPainter.SmoothPixmapTransform
        )

        # Get current OpenGL color
        gl_color = gl.glGetDoublev(gl.GL_CURRENT_COLOR)
        text_color = QColor(
            int(gl_color[2] * color[2]),
            int(gl_color[1] * color[1]),
            int(gl_color[0] * color[0]),
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
        self.track_vbo.delete()
        self.track_vbo = track_vbo(w, h)
