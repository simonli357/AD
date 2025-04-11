from PyQt5 import QtWidgets, QtCore
from OpenGL import GL as gl
from ..opengl.shader import ShaderRenderer

import glm


class CameraOverlay(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None, cam_widget=None):
        super().__init__(parent)
        self.setAttribute(QtCore.Qt.WA_TranslucentBackground, True)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.cam_widget = cam_widget

    def update_overlay(self):
        self.update()

    def initializeGL(self):
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glClearColor(0, 0, 0, 0)

        self.shader_renderer = ShaderRenderer()
        self.proj_mat = glm.ortho(0.0, self.width(), self.height(), 0.0, -1.0, 1.0)
        self.view_mat = glm.vec4(1.0)

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT)
        if not self.cam_widget.has_frame:
            self.shader_renderer.text_renderer.render_text("NO VIDEO", 320, 240, 1.0, (0.0, 1.0, 0.0), self.proj_mat)

    def resizeGL(self, w, h):
        super().resizeGL(w, h)
        self.proj_mat = glm.ortho(0.0, w, h, 0.0, -1.0, 1.0)
