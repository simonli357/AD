from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
import glm
from ..opengl.shader import ShaderRenderer

import numpy as np


class AnimationWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.stop_drawing = False
        self.main_window = self.parent()
        self.obj_dict = self.main_window.map_widget.object_dict

        self.cam_dist = 6.0
        self.cam_height = self.cam_dist * 2 / 3
        self.rotation = 0

    def render_widget(self) -> None:
        self.update()
        self.rotation = (self.rotation + 0.05) % (2 * np.pi)

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

        self.shader_renderer = ShaderRenderer()

    def paintGL(self):
        if self.stop_drawing:
            return

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        aspect = self.width() / self.height() if self.height() != 0 else 1.0
        self.proj_mat = glm.perspective(
            glm.radians(45.0),
            aspect,
            0.1,
            100.0
        )

        x = 0
        y = 0

        cam_x = x - self.cam_dist * np.cos(np.radians(0))
        cam_y = y - self.cam_dist * np.sin(np.radians(0))

        cam_pos = glm.vec3(cam_x, cam_y, self.cam_height)
        target_pos = glm.vec3(x, y, self.cam_height / 2)

        self.view_mat = glm.lookAt(
            cam_pos,
            target_pos,
            glm.vec3(0, 0, 1)
        )

        self.draw_detected_objects()

    def draw_detected_objects(self):
        obj = self.main_window.cam_widget.detected_objects
        if obj is None:
            return
        obj_detected = False
        for i in range(0, self.main_window.cam_widget.numObj):
            obj_type = self.obj_dict[obj[10 * i + 6]]
            x = obj[10 * i + 8]
            y = obj[10 * i + 7]
            distance = obj[10 * i + 4]
            self.shader_renderer.draw_road_object(obj_type, 0, 0, np.radians(180) + self.rotation, 16.0, self.view_mat, self.proj_mat)
            self.main_window.info_widget.update_obj_type(obj_type)
            self.main_window.info_widget.update_obj_dist(distance)
            self.main_window.info_widget.update_obj_pos((x, y))
            obj_detected = True
        if not obj_detected:
            self.shader_renderer.draw_void_symbol(0, 0, np.radians(180) + self.rotation, 1.5, self.view_mat, self.proj_mat)
            self.main_window.info_widget.update_obj_type()
            self.main_window.info_widget.update_obj_dist()
            self.main_window.info_widget.update_obj_pos()

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
