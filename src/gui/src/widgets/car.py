from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
from OpenGL import GLU as glu
from OpenGL.arrays import vbo
from .enums import MapData
import numpy as np
from collections import namedtuple
import os
import math

Model = namedtuple('Model', ['vertices', 'faces', 'vbo', 'vertex_count'])


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.stop_drawing = False
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
        self.car_model = None
        self.sign_model = None
        self.tf_light_model = None
        self.pedestrian_model = None

        current_dir = os.path.dirname(os.path.abspath(__file__))
        car_model_path = os.path.join(current_dir, 'assets', 'car.obj')
        sign_model_path = os.path.join(current_dir, 'assets', 'sign.obj')
        tf_light_model_path = os.path.join(current_dir, 'assets', 'tf_light.obj')
        pedestrian_model_path = os.path.join(current_dir, 'assets', 'pedestrian.obj')
        if os.path.exists(car_model_path):
            self.car_model = self.load_obj(car_model_path)
        if os.path.exists(sign_model_path):
            self.sign_model = self.load_obj(sign_model_path)
        if os.path.exists(tf_light_model_path):
            self.tf_light_model = self.load_obj(tf_light_model_path)
        if os.path.exists(pedestrian_model_path):
            self.pedestrian_model = self.load_obj(pedestrian_model_path)

    def set_steer(self, steer: float) -> None:
        self.steer = steer

    def set_car_data(self, yaw: float, x: float, y: float, z: float) -> None:
        if self.main_window.buttons_widget.started:
            dx = x - self.x_pos
            dy = y - self.y_pos
            displacement = np.sqrt(dx**2 + dy**2)
            self.main_window.map_widget.graphics_view.dist_traveled += displacement
            self.main_window.map_widget.graphics_view.set_distance_traveled()
            self.main_window.map_widget.graphics_view.update_visited_destinations(x, y)
        self.yaw = yaw
        self.x_pos = x
        self.y_pos = y
        self.z_pos = z

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
        grid_size_z = 15
        grid_size_x = 6
        step = 1
        grid_vertices = []
        for z in range(-grid_size_z, grid_size_z + 1, step):
            grid_vertices.extend([-grid_size_x, 0, z, grid_size_x, 0, z])
        for x in range(-grid_size_x, grid_size_x + 1, step):
            grid_vertices.extend([x, 0, -grid_size_z, x, 0, grid_size_z])

        self.grid_vbo = vbo.VBO(np.array(grid_vertices, dtype=np.float32))
        self.grid_vertex_count = len(grid_vertices) // 3

        # Initialize path node VBO
        third_z = 0.1 - (math.sqrt(3) / 2)
        path_node_vertices = [
            # Vertex 1: (-0.5, 0, 0.5) with color blue
            -0.1, 0.0, 0.1, 0, 0, 0,
            # Vertex 2: (0.5, 0, 0.5) with color green
            0.1, 0.0, 0.1, 0, 0, 0,
            # Vertex 3: (0.0, 0, third_z) with color red
            0.0, 0.0, -third_z, 0, 0, 0
        ]
        self.path_node_vbo = vbo.VBO(np.array(path_node_vertices, dtype=np.float32))

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
        glu.gluLookAt(0, 12, 16, 0, 0, 0, 0, 1, 0)

        # Draw grid
        self.draw_grid()

        gl.glTranslatef(0.5, 0, 0)

        # Draw car model
        self.draw_car_self((0, 1, 1, 1))

        # Draw detected objects if any
        self.draw_detected_object()

        # Draw predicted path
        self.draw_path()

        # Draw axes overlay
        self.draw_axes()

        self.qt_restore_gl_state()

        self.render_text(f'Yaw: {self.yaw:.2f}°', (0, 255, 255, 255), 10, 20)
        self.render_text(f'x: {self.x_pos:.2f}', (255, 0, 0, 255), 10, 45)
        self.render_text(f'y: {self.y_pos:.2f}', (0, 255, 0, 255), 10, 70)
        self.render_text(f'z: {self.z_pos:.2f}', (0, 0, 255, 255), 10, 95)

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

    def draw_grid(self):
        gl.glPushMatrix()
        gl.glColor3f(0.3, 0.3, 0.3)

        self.grid_vbo.bind()
        gl.glScalef(0.75, 0.75, 0.75)
        gl.glTranslatef(0.5, 2, -3)
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.grid_vbo)
        gl.glDrawArrays(gl.GL_LINES, 0, self.grid_vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.grid_vbo.unbind()

        gl.glPopMatrix()

    def draw_car_self(self, color: (float, float, float, float)):
        gl.glPushMatrix()
        gl.glScalef(0.5, 0.5, -0.5)
        gl.glTranslatef(0, -5, 0)
        gl.glRotatef(self.steer, 0, 1, 0)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(*color)
        gl.glLineWidth(0.01)

        self.car_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.car_model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.car_model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.car_model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)
        gl.glPopMatrix()

    def draw_car(self, x, y, color: (float, float, float, float)):
        gl.glPushMatrix()
        gl.glScalef(0.5, 0.5, -0.5)
        gl.glTranslatef(-x * 24, -5, y * 24)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(*color)
        gl.glLineWidth(0.01)

        self.car_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.car_model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.car_model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.car_model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)
        gl.glPopMatrix()

    def draw_sign(self, x, y, color: (float, float, float, float)):
        gl.glPushMatrix()
        gl.glScalef(0.03, 0.03, 0.03)
        gl.glTranslatef(-x * 400, -5, -y * 400 + 100)
        gl.glRotatef(210, 0, 1, 0)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(*color)
        gl.glLineWidth(0.01)

        self.sign_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.sign_model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.sign_model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.sign_model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)
        gl.glPopMatrix()

    def draw_traffic_light(self, x, y, color: (float, float, float, float)):
        gl.glPushMatrix()
        gl.glScalef(0.5, 0.5, 0.5)
        gl.glTranslatef(-x * 20, -5, -y * 20)
        gl.glRotatef(5, 0, 1, 0)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(*color)
        gl.glLineWidth(0.01)

        self.tf_light_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.tf_light_model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.tf_light_model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.tf_light_model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)
        gl.glPopMatrix()

    def draw_pedestrian(self, x, y, color: (float, float, float, float)):
        gl.glPushMatrix()
        gl.glScalef(6, 6, 6)
        gl.glTranslatef(-x * 5, -5, -y * 5 - 3)
        gl.glRotatef(45, 0, 1, 0)

        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_LINE)
        gl.glColor4f(*color)
        gl.glLineWidth(0.01)

        self.pedestrian_model.vbo.bind()
        gl.glEnableClientState(gl.GL_VERTEX_ARRAY)
        gl.glVertexPointer(3, gl.GL_FLOAT, 0, self.pedestrian_model.vbo)
        gl.glDrawArrays(gl.GL_TRIANGLES, 0, self.pedestrian_model.vertex_count)
        gl.glDisableClientState(gl.GL_VERTEX_ARRAY)
        self.pedestrian_model.vbo.unbind()

        gl.glDisable(gl.GL_BLEND)
        gl.glPopMatrix()

    def draw_path_node(self, x, y) -> None:
        gl.glPushMatrix()
        gl.glColor3f(1, 1, 0)
        self.path_node_vbo.bind()
        gl.glRotatef(-self.steer, 0, 1, 0)
        gl.glRotatef(-self.yaw, 0, 1, 0)
        gl.glTranslatef(-x, -2.5, -y)
        gl.glRotatef(self.yaw, 0, 1, 0)
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
        for i in range(0, len(waypoints) - 1, 4):
            x = waypoints[i] - (MapData.REAL_WORLD_WIDTH.value - self.x_pos)
            y = waypoints[i + 1] - (MapData.REAL_WORLD_HEIGHT.value - self.y_pos)
            self.draw_path_node(x * 4, y * 4)

    def draw_detected_object(self):
        obj = self.main_window.cam_widget.detected_objects
        if obj is None:
            return
        for i in range(0, self.main_window.cam_widget.numObj):
            obj_type = obj[10 * i + 6]
            x = obj[10 * i + 8]
            y = obj[10 * i + 7]
            if self.obj_dict[obj_type] == 'Car':
                self.draw_car(x, y, (0, 0.6, 0.6, 1))               # #009999
            if self.obj_dict[obj_type] == 'No Entry':
                self.draw_sign(x, y, (1, 0.4, 0, 1))                # #ff6600
            if self.obj_dict[obj_type] == 'Stopsign':
                self.draw_sign(x, y, (1, 0, 0, 1))                  # #ff0000
            if self.obj_dict[obj_type] == 'Oneway':
                self.draw_sign(x, y, (1, 1, 1, 1))                  # #ffffff
            if self.obj_dict[obj_type] == 'Highway Entrance':
                self.draw_sign(x, y, (0, 0.2, 0, 1))                # #004d00
            if self.obj_dict[obj_type] == 'Roundabout':
                self.draw_sign(x, y, (0, 0, 1, 1))                  # #0000ff
            if self.obj_dict[obj_type] == 'Parking':
                self.draw_sign(x, y, (0, 1, 0, 1))                  # #00ff00
            if self.obj_dict[obj_type] == 'Crosswalk':
                self.draw_sign(x, y, (1, 1, 0, 1))                  # #ffff00
            if self.obj_dict[obj_type] == 'Highway Exit':
                self.draw_sign(x, y, (0, 0.2, 0, 1))                # #004d00
            if self.obj_dict[obj_type] == 'Priority':
                self.draw_sign(x, y, (0, 0.4, 1, 1))                # #0066ff
            if self.obj_dict[obj_type] == 'Green Light':
                self.draw_traffic_light(x, y, (0, 1, 0, 1))         # #00ff00
            if self.obj_dict[obj_type] == 'Yellow Light':
                self.draw_traffic_light(x, y, (1, 1, 0, 1))         # #ffff00
            if self.obj_dict[obj_type] == 'Red Light':
                self.draw_traffic_light(x, y, (1, 0, 0, 1))         # #ff0000
            if self.obj_dict[obj_type] == 'Pedestrian':
                self.draw_pedestrian(x, y, (1, 0.2, 0.6, 1))        # #ff3399

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

        gl.glTranslatef(viewport[2] - 35, 25, 0)
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
