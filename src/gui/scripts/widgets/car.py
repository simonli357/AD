from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
from OpenGL import GLU as glu
from OpenGL.arrays import vbo
import numpy as np
from collections import namedtuple, deque
import os
import math

Model = namedtuple('Model', ['vertices', 'faces', 'vbo', 'vertex_count'])


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.main_window = self.parent()
        self.obj_dict = self.main_window.map_widget.object_dict

        fmt = self.format()
        fmt.setAlphaBufferSize(8)  # Enable alpha channel
        self.setFormat(fmt)

        self.steer = 0
        self.yaw = 0
        self.x_pos = 0
        self.y_pos = 0
        self.z_pos = 0

        self.grid_vbo = None
        self.path_node_vbo = None
        self.model = None
        self.car_model = None
        self.sign_model = None

        self.objects = deque([], 60)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        model_path = os.path.join(current_dir, 'assets', 'car.obj')
        sign_model_path = os.path.join(current_dir, 'assets', 'stop.obj')
        if os.path.exists(model_path):
            self.model = self.load_obj(model_path)
        if os.path.exists(sign_model_path):
            self.sign_model = self.load_obj(sign_model_path)

    def set_steer(self, steer: float) -> None:
        self.steer = steer

    def set_car_data(self, yaw: float, x: float, y: float, z: float) -> None:
        self.yaw = yaw
        self.x_pos = x
        self.y_pos = y
        self.z_pos = z

    def add_object(self, obj) -> None:
        self.objects.append(obj)

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
        grid_size_z = 8
        grid_size_x = 4
        step = 1
        grid_vertices = []
        for z in range(-grid_size_z, grid_size_z + 1, step):
            grid_vertices.extend([-grid_size_x, 0, z, grid_size_x, 0, z])
        for x in range(-grid_size_x, grid_size_x + 1, step):
            grid_vertices.extend([x, 0, -grid_size_z, x, 0, grid_size_z])

        self.grid_vbo = vbo.VBO(np.array(grid_vertices, dtype=np.float32))
        self.grid_vertex_count = len(grid_vertices) // 3

        # Initialize path node VBO
        third_z = 0.25 - (math.sqrt(3) / 2)
        path_node_vertices = [
            # Vertex 1: (-0.5, 0, 0.5) with color blue
            -0.25, 0.0, 0.25, 0, 0, 0,
            # Vertex 2: (0.5, 0, 0.5) with color green
            0.25, 0.0, 0.25, 0, 0, 0,
            # Vertex 3: (0.0, 0, third_z) with color red
            0.0, 0.0, -third_z, 0, 0, 0
        ]
        self.path_node_vbo = vbo.VBO(np.array(path_node_vertices, dtype=np.float32))

    def paintGL(self):
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
        glu.gluLookAt(0, 15, 15, 0, 0, 0, 0, 1, 0)

        # Draw grid
        self.draw_grid()

        # Draw car model
        if self.model:
            gl.glPushMatrix()
            gl.glScalef(0.5, 0.5, -0.5)
            gl.glTranslatef(0, -5, 0)
            gl.glRotatef(self.steer, 0, 1, 0)
            self.draw_model()
            gl.glPopMatrix()

        # Draw detected objects if any
        self.draw_detected_object()

        # Draw predicted path
        self.draw_path()

        # Draw axes overlay
        self.draw_axes()

        self.qt_restore_gl_state()

        self.render_text(f'Yaw: {self.yaw:.2f}°', (0, 255, 255, 255), 10, 20)
        self.render_text(f'x: {self.x_pos:.2f}', (255, 0, 0, 255), 10, 35)
        self.render_text(f'y: {self.y_pos:.2f}', (0, 255, 0, 255), 10, 50)
        self.render_text(f'z: {self.z_pos:.2f}', (0, 0, 255, 255), 10, 65)

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
        font = QFont("Arial", 8)
        font.setBold(True)
        font.setStyleStrategy(QFont.PreferAntialias)

        # Account for high-DPI scaling
        scale_factor = self.devicePixelRatio()
        painter.scale(1 / scale_factor, 1 / scale_factor)
        font.setPixelSize(12 * scale_factor)

        painter.setPen(text_color)
        painter.setFont(font)
        painter.drawText(int(x * scale_factor),
                         int(y * scale_factor),
                         text)
        painter.end()

    def draw_grid(self):
        gl.glPushMatrix()
        gl.glColor3f(0.3, 0.3, 0.3)

        self.grid_vbo.bind()
        gl.glTranslatef(0, 2, 0)
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
        gl.glColor4f(0, 1, 1, 1.0)
        gl.glLineWidth(0.01)

        self.model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)

    def draw_car(self):
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(1, 0, 0, 1.0)
        gl.glLineWidth(0.01)

        self.model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)

    def draw_sign(self):
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(1, 1, 0, 1.0)
        gl.glLineWidth(0.01)

        self.sign_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.sign_model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)

    def draw_path_node(self, x, y) -> None:
        gl.glPushMatrix()
        gl.glColor3f(1, 1, 0)

        self.path_node_vbo.bind()
        gl.glTranslatef(x, 0, y)
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.path_node_vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, 3)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.path_node_vbo.unbind()

        gl.glPopMatrix()

    def draw_path(self) -> None:
        waypoints = self.main_window.map_widget.waypoints
        if waypoints is None:
            return
        for i in range(0, len(waypoints) - 1, 8):
            x = waypoints[i]
            y = waypoints[i + 1]
            self.draw_path_node(x, y)

    def draw_detected_object(self):
        obj = self.main_window.map_widget.detected_objects
        if obj is None:
            return
        for i in range(1, len(obj)):
            obj_type = obj[6]
            if self.obj_dict[obj_type] == 'Car':
                x = obj[8] * 12
                y = obj[7] * 12
                if self.model:
                    gl.glPushMatrix()
                    gl.glScalef(0.5, 0.5, -0.5)
                    gl.glTranslatef(-x, 0, y)
                    self.draw_car()
                    gl.glPopMatrix()
            elif self.obj_dict[obj_type] == 'Stopsign':
                x = obj[8] * 150
                y = obj[7] * 150
                if self.sign_model:
                    gl.glPushMatrix()
                    gl.glScalef(-0.03, 0.03, 0.03)
                    gl.glRotatef(210, 0, 1, 0)
                    gl.glTranslatef(-x, 0, y)
                    self.draw_sign()
                    gl.glPopMatrix()

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

        gl.glTranslatef(viewport[2] - 20, 35, 0)
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
