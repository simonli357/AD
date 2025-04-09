from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from OpenGL import GL as gl
import glm
from ..enums import MapData, NamedColor
from ..opengl.shader import ShaderRenderer
from ..opengl.gt import GTRenderer

import numpy as np


class CarWidget(QtWidgets.QOpenGLWidget):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setAttribute(QtCore.Qt.WA_AlwaysStackOnTop, True)
        self.stop_drawing = False
        self.main_window = self.parent()

        self.yaw = 0
        self.x_pos = 11.75
        self.y_pos = MapData.REAL_WORLD_HEIGHT.value - 2.05
        self.z_pos = 0

        self.cam_dist = 16.0
        self.cam_height = self.cam_dist * 2 / 3

        self.updated_dest_size = 4.0

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
        self.destinations_renderer = GTRenderer(self.shader_renderer.destination_model, 'Destination')
        self.update_destinations()

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

        self.destinations_renderer.draw(self.proj_mat, self.view_mat)
        self.draw_path_nodes(self.main_window.map_widget.waypoints)
        self.draw_detected_objects(self.main_window.map_widget.detected_data, self.main_window.map_widget.road_msg_dict, self.main_window.map_widget.object_dict)

        # Grow visited destination until out of sight
        if self.updated_dest_size < 4.0:
            self.shader_renderer.draw_destination(self.current_destx, self.current_desty, 0, self.updated_dest_size, self.view_mat, self.proj_mat)
            self.updated_dest_size += 0.01

    def update_visited_destination(self, x_visited, y_visited):
        self.updated_dest_size = 2.1
        self.current_destx, self.current_desty = self.get_gl_coords(x_visited, y_visited)

    def draw_detected_objects(self, detected_data, road_msg_dict, object_dict):
        if detected_data is None or len(detected_data) == 0:
            return
        for i in range(len(detected_data)):
            obj_type = detected_data[i, road_msg_dict['type']]
            x_real = detected_data[i, road_msg_dict['x']]
            y_real = detected_data[i, road_msg_dict['y']]
            orientation = detected_data[i, road_msg_dict['orientation']]

            # Convert map coordinates to pixel coordinates
            x, y = self.get_gl_coords(x_real, MapData.REAL_WORLD_HEIGHT.value - y_real)
            # orientation = 2 * np.pi - orientation
            orientation = - orientation

            if object_dict[obj_type] == 'Car' and i == 0:
                continue
            elif object_dict[obj_type] == 'Car':
                self.shader_renderer.draw_car(x, y, orientation, NamedColor.RED, 0.20, self.view_mat, self.proj_mat)
            else:
                self.shader_renderer.draw_road_object(object_dict[obj_type], x, y, orientation, 16.0, self.view_mat, self.proj_mat)

    def draw_path_nodes(self, waypoints):
        if waypoints is None or len(waypoints) < 2:
            return
        x1, y1 = self.get_gl_coords(waypoints[0], MapData.REAL_WORLD_HEIGHT.value - waypoints[1])
        angle = 0
        for i in range(0, len(waypoints) - 1, 4):
            if i + 3 > len(waypoints):
                self.shader_renderer.draw_triangle(x1, y1, 0.1, angle, (1, 1), (1.0, 1.0, 0.0, 1.0), self.view_mat, self.proj_mat)
            else:
                x2, y2 = self.get_gl_coords(waypoints[i + 2], MapData.REAL_WORLD_HEIGHT.value - waypoints[i + 3])
                dx = x2 - x1
                dy = y2 - y1
                angle = np.arctan2(dy, dx + (1e-5)) - np.pi / 2
                self.shader_renderer.draw_triangle(x1, y1, 0.1, angle, (1, 1), (1.0, 1.0, 0.0, 1.0), self.view_mat, self.proj_mat)
                x1, y1 = x2, y2

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

    def update_destinations(self):
        self.destinations_renderer.update_data(self.main_window.map_widget.data.iterrows(), self.width(), self.height())

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
        self.update_destinations()
