from PyQt5 import QtWidgets
from OpenGL import GL as gl
from OpenGL import GLU as glu


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.yaw = 0

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
        glu.gluLookAt(3, 3, 3, 0, 0, 0, 0, 1, 0)

        self.draw_axes()

        gl.glPushMatrix()
        gl.glRotatef(self.yaw, 0, 1, 0)  # Apply yaw rotation around Y-axis
        self.draw_cube()
        gl.glPopMatrix()

    def draw_axes(self):
        gl.glBegin(gl.GL_LINES)
        # X-axis (Red)
        gl.glColor3f(1, 0, 0)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(1, 0, 0)
        # Y-axis (Green)
        gl.glColor3f(0, 1, 0)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 1, 0)
        # Z-axis (Blue)
        gl.glColor3f(0, 0, 1)
        gl.glVertex3f(0, 0, 0)
        gl.glVertex3f(0, 0, 1)
        gl.glEnd()

    def draw_cube(self):
        vertices = [
            (-0.5, -0.5, 0.5), (0.5, -0.5, 0.5),
            (0.5, 0.5, 0.5), (-0.5, 0.5, 0.5),
            (-0.5, -0.5, -0.5), (0.5, -0.5, -0.5),
            (0.5, 0.5, -0.5), (-0.5, 0.5, -0.5)
        ]
        faces = [
            (0, 1, 2, 3), (4, 5, 6, 7),
            (0, 3, 7, 4), (1, 2, 6, 5),
            (0, 1, 5, 4), (3, 2, 6, 7)
        ]
        colors = [
            (1, 0, 0), (0, 1, 0),
            (0, 0, 1), (1, 1, 0),
            (0, 1, 1), (1, 0, 1)
        ]
        gl.glBegin(gl.GL_QUADS)
        for i in range(6):
            gl.glColor3fv(colors[i])
            for j in faces[i]:
                gl.glVertex3fv(vertices[j])
        gl.glEnd()
