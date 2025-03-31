from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
from OpenGL.arrays import vbo
from collections import namedtuple
from .renderer import draw_track, draw_waypoints, draw_car, draw_grid
from .vbos import ellipse_vbo, grid_vbo
from ..enums import BarcaMapData

import os
import numpy as np

Model = namedtuple('Model', ['vertices', 'faces', 'vbo', 'vertex_count'])


class BarcaWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.stop_drawing = False
        self.main_window = self.parent()
        self.car_widget = self.main_window.car_widget
        self.pan_x = 0
        self.pan_y = 0
        self.max_zoom = 40
        self.zoom_level = self.max_zoom
        self.last_mouse_pos = None

        fmt = self.format()
        fmt.setAlphaBufferSize(8)  # Enable alpha channel
        self.setFormat(fmt)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        track_model_path = os.path.join(current_dir, 'assets', 'track.obj')
        car_model_path = os.path.join(current_dir, 'assets', 'car.obj')
        if os.path.exists(track_model_path):
            self.track_model = self.load_obj(track_model_path)
        if os.path.exists(car_model_path):
            self.car_model = self.load_obj(car_model_path)

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

        # initialize waypoint vbo
        self.wp_vbo = ellipse_vbo(0.05, 0.05)
        grid_model = grid_vbo()
        self.grid_vbo = grid_model[0]
        self.grid_vertex_count = grid_model[1]

    def paintGL(self):
        if self.stop_drawing:
            return

        self.qt_save_gl_state()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # Set up projection matrix
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()

        # aspect = self.width() / self.height() if self.height() != 0 else 1.0
        # glu.gluPerspective(45, aspect, 0.1, 100.0)

        aspect = self.width() / self.height() if self.height() != 0 else 1.0
        half_zoom = self.zoom_level * 0.5
        left = self.pan_x - half_zoom * aspect
        right = self.pan_x + half_zoom * aspect
        bottom = self.pan_y - half_zoom
        top = self.pan_y + half_zoom
        gl.glOrtho(left, right, bottom, top, -100, 100)  # Near and far planes

        # Set up view matrix
        gl.glMatrixMode(gl.GL_MODELVIEW)
        gl.glLoadIdentity()

        # Camera
        # glu.gluLookAt(0, 0, self.zoom_level, 0, 0, 0, 0, 1, 0)  # Camera at (0,0,zoom_level)
        # gl.glTranslatef(self.pan_x, self.pan_y, 0)

        # Global Transforms
        gl.glRotatef(25, 0.0, 0.0, 1.0)
        gl.glTranslatef(-6.5, 2, 0)

        draw_track(self.track_model)
        # Counter map offset
        gl.glTranslatef(BarcaMapData.MAP_CENTER_X.value, -BarcaMapData.MAP_CENTER_Y.value, 0)
        draw_grid(self.grid_vbo, self.grid_vertex_count)
        gl.glTranslatef(-2, 6, 0)

        # Draw objects
        draw_waypoints(self.main_window.map_widget.state_refs_np, self.wp_vbo)
        draw_car(self.car_widget.x_pos, self.car_widget.y_pos, self.car_widget.yaw, self.car_model)

        self.qt_restore_gl_state()

        # self.render_text(f'Yaw: {self.yaw:.2f}°', (0, 255, 255, 255), 10, 20)
        # self.render_text(f'x: {self.x_pos:.2f}', (255, 0, 0, 255), 10, 45)
        # self.render_text(f'y: {self.y_pos:.2f}', (0, 255, 0, 255), 10, 70)
        # self.render_text(f'z: {self.z_pos:.2f}', (0, 0, 255, 255), 10, 95)

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

    ###############
    # Events
    ###############

    def mousePressEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton and self.last_mouse_pos is not None:
            # Prevent panning when at initial zoom
            if self.zoom_level >= self.max_zoom:
                return

            dx = event.pos().x() - self.last_mouse_pos.x()
            dy = event.pos().y() - self.last_mouse_pos.y()
            self.last_mouse_pos = event.pos()

            widget_height = self.height()
            if widget_height == 0:
                widget_height = 1

            aspect = self.width() / widget_height
            scale = self.zoom_level / widget_height

            # Calculate proposed pan changes
            new_pan_x = self.pan_x - dx * scale
            new_pan_y = self.pan_y + dy * scale

            # Calculate content boundaries based on initial zoom
            half_span_x = (self.max_zoom - self.zoom_level) * aspect / 2
            half_span_y = (self.max_zoom - self.zoom_level) / 2

            # Clamp pan values to content boundaries
            self.pan_x = max(-half_span_x, min(half_span_x, new_pan_x))
            self.pan_y = max(-half_span_y, min(half_span_y, new_pan_y))

            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta != 0:
            new_zoom = self.zoom_level - delta * 0.015
            new_zoom = max(8, min(self.max_zoom, new_zoom))
            if new_zoom != self.zoom_level:
                self.zoom_level = new_zoom
                # Reset pan when returning to initial zoom
                if self.zoom_level == self.max_zoom:
                    self.pan_x = 0
                    self.pan_y = 0
                self.update()
