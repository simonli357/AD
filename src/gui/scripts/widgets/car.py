from PyQt5 import QtWidgets
from OpenGL import GL as gl
from OpenGL import GLU as glu
from collections import namedtuple

import os
Model = namedtuple('Model', ['vertices', 'faces'])


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.yaw = 0
        current_dir = os.path.dirname(os.path.abspath(__file__))
        assets_dir = os.path.join(current_dir, 'assets')
        model_path = os.path.join(assets_dir, 'car.obj')
        self.model = self.load_obj(model_path)

    def load_obj(self, model_path) -> Model:
        vertices = []
        faces = []
        with open(model_path, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    vertex = list(map(float, line.strip().split()[1:4]))
                    vertices.append(vertex)
                elif line.startswith('f '):
                    face = [int(v.split('/')[0]) - 1 for v in line.strip().split()[1:]]
                    faces.append(face)
        return Model(vertices=vertices, faces=faces)

    def set_yaw(self, yaw: float) -> None:
        self.yaw = yaw
        self.update()

    def initializeGL(self):
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)

    def resizeGL(self, w, h):
        gl.glViewport(0, 0, w, h)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = w / h if h != 0 else 1.0
        glu.gluPerspective(45, aspect, 0.1, 100.0)
        gl.glMatrixMode(gl.GL_MODELVIEW)

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        glu.gluLookAt(8, 8, 8, 0, 0, 0, 0, 1, 0)  # Camera position
        if self.model:
            gl.glPushMatrix()
            gl.glRotatef(self.yaw, 0, 1, 0)  # Yaw rotation
            self.draw_model()
            gl.glPopMatrix()
        self.draw_axes()

    def draw_axes(self):
        gl.glBegin(gl.GL_LINES)
        # X-axis (Red)
        gl.glColor3f(6, 0, 0)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(6, 0, 0)
        # Y-axis (Green)
        gl.glColor3f(0, 6, 0)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 6, 0)
        # Z-axis (Blue)
        gl.glColor3f(0, 0, 6)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 0, 6)
        gl.glEnd()

    def draw_model(self):
        # gl.glColor3f(1, 0, 0)  # Default color
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(0.6, 0, 1, 0.3)
        gl.glLineWidth(1.5)
        gl.glBegin(gl.GL_TRIANGLES)
        for face in self.model.faces:
            for vertex_idx in face:
                gl.glVertex3fv(self.model.vertices[vertex_idx])
        gl.glEnd()
