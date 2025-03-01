from PyQt5 import QtWidgets
from PyQt5.Qt import QPainter, QColor
from OpenGL import GL as gl
from OpenGL import GLU as glu
from OpenGL.arrays import vbo
import numpy as np
from collections import namedtuple
import os

Model = namedtuple('Model', ['vertices', 'faces', 'vbo', 'vertex_count'])


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMaximumHeight(185)
        self.yaw = 0
        self.grid_vbo = None
        self.model = None

        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'assets', 'car.obj')
        if os.path.exists(model_path):
            self.model = self.load_obj(model_path)

    def load_obj(self, model_path) -> Model:
        vertices = []
        faces = []
        with open(model_path, 'r') as f:
            for line in f:
                if line.startswith('v '):
                    vertices.append(list(map(float, line.strip().split()[1:4])))
                elif line.startswith('f '):
                    faces.append([int(v.split('/')[0]) - 1 for v in line.strip().split()[1:]])

        # Convert to flat array of vertices
        vertex_data = []
        for face in faces:
            for v_idx in face:
                vertex_data.extend(vertices[v_idx])

        vertex_array = np.array(vertex_data, dtype=np.float32)
        model_vbo = vbo.VBO(vertex_array)
        return Model(vertices=vertices, faces=faces, vbo=model_vbo,
                     vertex_count=len(vertex_data) // 3)

    def initializeGL(self):
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)

        # Initialize grid VBO
        grid_size = 8
        step = 2
        grid_vertices = []
        for z in range(-grid_size, grid_size + 1, step):
            grid_vertices.extend([-grid_size, 0, z, grid_size, 0, z])
        for x in range(-grid_size, grid_size + 1, step):
            grid_vertices.extend([x, 0, -grid_size, x, 0, grid_size])

        self.grid_vbo = vbo.VBO(np.array(grid_vertices, dtype=np.float32))
        self.grid_vertex_count = len(grid_vertices) // 3

    def paintGL(self):
        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # Set up projection matrix
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = self.width() / self.height() if self.height() != 0 else 1.0
        glu.gluPerspective(45, aspect, 0.1, 100.0)

        # Set up view matrix
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        glu.gluLookAt(10, 8, 10, 0, 0, 0, 0, 1, 0)

        # Draw grid
        self.draw_grid()

        # Draw car model
        if self.model:
            gl.glPushMatrix()
            gl.glScalef(0.8, 0.8, -0.8)
            gl.glRotatef(-self.yaw, 0, 1, 0)
            gl.glTranslatef(1, -2, -1)
            self.draw_model()
            gl.glPopMatrix()

        # Draw axes overlay
        self.draw_axes()

    def draw_grid(self):
        gl.glPushMatrix()
        gl.glTranslatef(0, -2.8, 0)  # Adjusted Y position
        gl.glColor3f(0.3, 0.3, 0.3)

        self.grid_vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.grid_vbo)
        gl.glDrawArrays(gl.GL_LINES, 0, self.grid_vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.grid_vbo.unbind()

        gl.glPopMatrix()

    def draw_model(self):
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(0, 1, 1, 0.3)
        gl.glLineWidth(1.5)

        self.model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)

    def draw_axes(self):
        gl.glPushAttrib(gl.GL_ENABLE_BIT)
        gl.glPushMatrix()

        viewport = gl.glGetIntegerv(gl.GL_VIEWPORT)
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPushMatrix()
        gl.glLoadIdentity()
        gl.glOrtho(0, viewport[2], viewport[3], 0, -1, 1)

        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

        gl.glTranslatef(viewport[2] - 50, 20, 0)
        gl.glScalef(20, 20, 20)

        gl.glDisable(gl.GL_DEPTH_TEST)

        gl.glBegin(gl.GL_LINES)
        # Z-axis (Blue)
        gl.glColor3f(0, 0, 1)
        gl.glVertex2f(0, 0)
        gl.glVertex2f(-1, 0.5)
        # Y-axis (Green)
        gl.glColor3f(0, 1, 0)
        gl.glVertex2f(0, 0)
        gl.glVertex2f(0, -1)
        # X-axis (Red)
        gl.glColor3f(1, 0, 0)
        gl.glVertex2f(0, 0)
        gl.glVertex2f(1.0, 0.35)
        gl.glEnd()

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPopMatrix()
        gl.glPopAttrib()
        gl.glEnable(gl.GL_DEPTH_TEST)

    def set_yaw(self, yaw: float) -> None:
        if self.yaw != yaw:
            self.yaw = yaw
            self.update()

    def resizeGL(self, w, h):
        gl.glViewport(0, 0, w, h)
