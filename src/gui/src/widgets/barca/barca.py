from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
from OpenGL import GLU as glu
from OpenGL.arrays import vbo
from collections import namedtuple

import os
import numpy as np

Model = namedtuple('Model', ['vertices', 'faces', 'vbo', 'vertex_count'])


class BarcaWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.stop_drawing = False
        self.main_window = self.parent()

        fmt = self.format()
        fmt.setAlphaBufferSize(8)  # Enable alpha channel
        self.setFormat(fmt)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        barca_model_path = os.path.join(current_dir, 'assets', 'model.obj')
        if os.path.exists(barca_model_path):
            self.track_model = self.load_obj(barca_model_path)

    def render_widget(self) -> None:
        self.update()

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
        grid_size_x = 35
        grid_size_y = 12
        step = 1
        grid_vertices = []

        # Create the grid: Horizontal lines (parallel to the X axis)
        for x in range(-grid_size_x, grid_size_x + 1, step):
            grid_vertices.extend([x, -grid_size_y, 0, x, grid_size_y + 1, 0])

        # Create the grid: Vertical lines (parallel to the Y axis)
        for y in range(-grid_size_y, grid_size_y + 2, step):
            grid_vertices.extend([-grid_size_x, y, 0, grid_size_x, y, 0])

        self.grid_vbo = vbo.VBO(np.array(grid_vertices, dtype=np.float32))
        self.grid_vertex_count = len(grid_vertices) // 3

    def paintGL(self):
        if self.stop_drawing:
            return

        self.qt_save_gl_state()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # Set up projection matrix
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()
        aspect = self.width() / self.height() if self.height() != 0 else 1.0
        glu.gluPerspective(45, aspect, 0.1, 100.0)

        # Set up view matrix
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()
        glu.gluLookAt(0, 0, 55, 0, 0, 0, 0, 1, 0)

        # Draw axes overlay
        self.draw_axes()

        # Translate everything to the left
        gl.glTranslatef(-6, 0, 0)

        # Draw barca track
        self.draw_track()

        self.qt_restore_gl_state()

        # self.render_text(f'Yaw: {self.yaw:.2f}°', (0, 255, 255, 255), 10, 20)
        # self.render_text(f'x: {self.x_pos:.2f}', (255, 0, 0, 255), 10, 45)
        # self.render_text(f'y: {self.y_pos:.2f}', (0, 255, 0, 255), 10, 70)
        # self.render_text(f'z: {self.z_pos:.2f}', (0, 0, 255, 255), 10, 95)

    def draw_track(self):
        gl.glPushMatrix()

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(0.3, 0.3, 0.3, 0.3)
        gl.glLineWidth(0.01)

        self.track_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.track_model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.track_model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.track_model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)
        gl.glPopMatrix()

    def qt_save_gl_state(self):
        gl.glPushClientAttrib(gl.GL_CLIENT_ALL_ATTRIB_BITS)
        gl.glPushAttrib(gl.GL_ALL_ATTRIB_BITS)
        gl.glMatrixMode(gl.GL_TEXTURE)
        gl.glPushMatrix()
        gl.glLoadIdentity()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPushMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPushMatrix()

        gl.glShadeModel(gl.GL_FLAT)
        gl.glDisable(gl.GL_CULL_FACE)
        gl.glDisable(gl.GL_LIGHTING)
        gl.glDisable(gl.GL_STENCIL_TEST)
        gl.glDisable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

    def qt_restore_gl_state(self):
        gl.glMatrixMode(gl.GL_TEXTURE)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPopMatrix()
        gl.glPopAttrib()
        gl.glPopClientAttrib()

    def render_text(self, text, color: (int, int, int, int), x, y) -> None:
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
        font.setPixelSize(18 * scale_factor)

        painter.setPen(text_color)
        painter.setFont(font)
        painter.drawText(int(x * scale_factor),
                         int(y * scale_factor),
                         text)
        painter.end()

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

        gl.glTranslatef(viewport[2] - 75, 75, 0)
        gl.glScalef(20, 20, 20)

        gl.glDisable(gl.GL_DEPTH_TEST)

        gl.glBegin(gl.GL_LINES)
        # Z-axis (Red)
        gl.glColor3f(1, 0, 0)
        gl.glVertex2f(0, 0)
        gl.glVertex2f(-1, 0.0)
        # Y-axis (Blue)
        gl.glColor3f(0, 0, 1)
        gl.glVertex2f(0, 0)
        gl.glVertex2f(0, -1)
        # X-axis (Green)
        gl.glColor3f(0, 1, 0)
        gl.glVertex2f(0, 0)
        gl.glVertex2f(-0.5, -0.5)
        gl.glEnd()

        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glPopMatrix()
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glPopMatrix()
        gl.glPopAttrib()
        gl.glEnable(gl.GL_DEPTH_TEST)

    def resizeGL(self, w, h):
        gl.glViewport(0, 0, w, h)

    def cleanup_gl_resources(self):
        self.stop_drawing = True
        try:
            self.makeCurrent()
            if self.grid_vbo is not None:
                self.grid_vbo.delete()
                self.grid_vbo = None
            if self.path_node_vbo is not None:
                self.path_node_vbo.delete()
                self.path_node_vbo = None
            models = [
                self.car_model, self.sign_model,
                self.tf_light_model, self.pedestrian_model
            ]
            for model in models:
                if model and model.vbo:
                    model.vbo.delete()
                    model.vbo = None
            gl.glFlush()
            self.doneCurrent()
        except Exception:
            pass

    def __del__(self):
        self.cleanup_gl_resources()

    def deleteLater(self):
        self.cleanup_gl_resources()
        super().deleteLater()
