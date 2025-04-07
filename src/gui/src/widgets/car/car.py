from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
import glm
from ..enums import MapData, NamedColor
from ..opengl.shader import ShaderRenderer

import numpy as np


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.stop_drawing = False
        self.main_window = self.parent()

        self.steer = 0
        self.yaw = 0
        self.x_pos = 11.75
        self.y_pos = MapData.REAL_WORLD_HEIGHT.value - 2.05
        self.z_pos = 0

        self.cam_dist = 16.0
        self.cam_height = self.cam_dist * 2 / 3

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

        x, y = self.get_gl_coords(self.x_pos, self.y_pos)

        cam_x = x - self.cam_dist * np.cos(np.radians(self.yaw))
        cam_y = y - self.cam_dist * np.sin(np.radians(self.yaw))

        cam_pos = glm.vec3(cam_x, cam_y, self.cam_height)
        target_pos = glm.vec3(x, y, 0)

        self.view_mat = glm.lookAt(
            cam_pos,
            target_pos,
            glm.vec3(0, 0, 1)
        )

        self.shader_renderer.draw_car(
            x=x,
            y=y,
            yaw=np.radians(self.yaw),
            scale=0.20,
            color=NamedColor.WHITE,
            view_matrix=self.view_mat,
            proj_matrix=self.proj_mat
        )

        self.shader_renderer.draw_texture(
            mat=self.shader_renderer.bfmc_track_model,
            x=0.0,
            y=0.0,
            z=0,
            scale=(self.width(), self.height()),
            view_matrix=self.view_mat,
            proj_matrix=self.proj_mat
        )

        self.draw_gt(x, y)

    def draw_gt(self, car_x, car_y):
        for index, row in self.main_window.map_widget.data.iterrows():
            entity_type, orientation = row['Type'], row['Orientation']

            x, y = self.get_gl_coords(row['X'], MapData.REAL_WORLD_HEIGHT.value - row['Y'])
            dx = x - car_x
            dy = y - car_y
            dist = np.hypot(dx, dy)
            if dist > 40:
                continue

            # orientation = 2 * np.pi - orientation
            orientation = - orientation

            if entity_type == 'Car':
                self.shader_renderer.draw_car(x, y, -orientation, NamedColor.BLUE, 0.20, self.view_mat, self.proj_mat)
            elif entity_type == 'Destination':
                self.shader_renderer.draw_destination(x, y, orientation, 2.0, self.view_mat, self.proj_mat)
            else:
                if entity_type == 'Sign':
                    self.shader_renderer.draw_road_object('Stopsign', x, y, orientation, 16.0, self.view_mat, self.proj_mat)
                self.shader_renderer.draw_road_object(entity_type, x, y, orientation, 16.0, self.view_mat, self.proj_mat)

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

    def get_gl_coords(self, real_x, real_y):
        # Convert real-world to OpenGL world coordinates
        world_x = real_x / MapData.REAL_WORLD_WIDTH.value * self.width()
        world_y = (MapData.REAL_WORLD_HEIGHT.value - real_y) / MapData.REAL_WORLD_HEIGHT.value * self.height()
        return world_x, world_y

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
