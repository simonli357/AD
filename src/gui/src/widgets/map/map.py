from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from std_srvs.srv import TriggerResponse
from OpenGL import GL as gl
from .view import HidableOverlay
from .opengl.waypoints import WaypointsRenderer
from ..opengl.renderer import GlobalRenderer
from ..opengl.shader import ShaderRenderer
from ..enums import MapData

import pandas as pd
import os
import time
import numpy as np
import math
import glm


class MapWidget(QtWidgets.QOpenGLWidget):
    update_map_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.server = self.main_window.server
        self.cursor_coords = []
        self.cursor_x = 3.86
        self.cursor_y = 3.62

        self.show_mouse = True
        self.show_signs = False
        self.show_lanes = False
        self.show_cars = False
        self.show_destinations = True
        self.show_path = True
        self.show_nodes = False
        self.show_gt = True
        self.state_refs_np = None
        self.attributes_np = None
        self.sign_size = 20

        self.road_msg_length = 7
        self.road_msg_dict = {
            'type': 0,
            'x': 1,
            'y': 2,
            'orientation': 3,
            'speed': 4,
            'confidence': 5,
            'z': 6
        }

        self.detected_data = None
        self.waypoints = None
        self.numObj = 0
        self.detected_objects = np.zeros(10)

        current_dir = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
        self.assets_dir = os.path.join(current_dir, 'assets')
        self.data = pd.read_csv(os.path.join(self.assets_dir, 'coordinates_with_context.csv'))

        self.object_dict = {
            0: "Oneway",
            1: "Highway Entrance",
            2: "Stopsign",
            3: "Roundabout",
            4: "Parking",
            5: "Crosswalk",
            6: "No Entry",
            7: "Highway Exit",
            8: "Priority",
            9: "Light",
            10: "Block",
            11: "Pedestrian",
            12: "Car",
            13: "Green Light",
            14: "Yellow Light",
            15: "Red Light",
            16: "Sign"
        }
        self.reverse_object_dict = {v: k for k, v in self.object_dict.items()}

        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        self.setMouseTracking(True)
        self.stop_drawing = False
        self.current_mouse_pos = None
        self.last_mouse_pos = None
        self.show_mouse = True

        self.view_center = glm.vec2(0, 0)
        self.view_zoom = 1.0
        self.drag_start = None
        self.base_view_center = glm.vec2(self.view_center)

        self.waypoints_renderer = WaypointsRenderer()

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.track_model_path = os.path.join(current_dir, 'assets', 'track.png')
        self.car_model_path = os.path.join(current_dir, 'assets', 'car.obj')

        self.sign_images = []
        self.sign_images.append(os.path.join(current_dir, 'assets', 'oneway.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'highway_entrance.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'stopsign.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'roundabout.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'parking.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'crosswalk.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'noentry.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'highway_exit.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'priority.png'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'trafficlight.png'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'roadblock.png'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'pedestrian.png'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'car.jpg'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'trafficlight_green.png'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'trafficlight_yellow.png'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'trafficlight_red.png'))
        self.sign_images.append(os.path.join(current_dir, 'assets', 'stopsign2.jpg'))

        self.setup_ui()

    def setup_ui(self):
        self.cursor_coords_label = QtWidgets.QLabel(self)
        self.cursor_coords_label.setStyleSheet("""
            border: none;
            background-color: rgba(0, 0, 0, 0.7);
            color: #00ff00;
            font-size: 16px;
        """)
        self.cursor_coords_label.hide()
        self.cursor_coords_label.setAlignment(QtCore.Qt.AlignCenter)
        self.run_statistics = HidableOverlay(self)
        self.run_statistics.setFixedSize(
            int(256),
            int(144)
        )

    def get_key_from_value(self, value):
        return self.reverse_object_dict.get(value, None)

    def render_widget(self) -> None:
        self.update()

    def initializeGL(self):
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)
        gl.glEnable(gl.GL_CULL_FACE)
        gl.glCullFace(gl.GL_BACK)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glDisable(gl.GL_BLEND)          # Disable unless transparency needed
        gl.glDisable(gl.GL_LINE_SMOOTH)    # Avoid anti-aliasing overhead
        gl.glDisable(gl.GL_POLYGON_SMOOTH)
        gl.glDisable(gl.GL_MULTISAMPLE)    # Disable MSAA if not used
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)  # Fastest mode
        gl.glShadeModel(gl.GL_FLAT)        # Faster than GL_SMOOTH if applicable

        self.renderer = GlobalRenderer()
        self.shader_renderer = ShaderRenderer()

    def paintGL(self):
        if self.stop_drawing:
            return
        if self.view_zoom == 1:
            self.view_center = glm.vec2(0, 0)

        self.update_mouse_pos()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        zoom_factor = 1.0 / self.view_zoom
        half_width = self.width() * zoom_factor / 2
        half_height = self.height() * zoom_factor / 2

        proj_mat = glm.ortho(
            -half_width, half_width,
            -half_height, half_height,
            0.1, 100.0
        )

        view_mat = glm.lookAt(
            glm.vec3(self.view_center.x, self.view_center.y, 1.0),
            glm.vec3(self.view_center.x, self.view_center.y, 0.0),
            glm.vec3(0.0, 1.0, 0.0)
        )

        self.shader_renderer.draw_texture(
            mat=self.shader_renderer.bfmc_track_model,
            x=-self.width() / 2,
            y=-self.height() / 2,
            z=-1.1,
            scale=(self.width(), self.height()),
            view_matrix=view_mat,
            proj_matrix=proj_mat
        )

        if self.view_zoom == 1.0:
            self.draw_legend(self.width() / 2.7, self.height() / 3)

    def draw_legend(self, x, y):
        offset = 30
        y_offset = 0
        self.render_text("◈", 32, (0, 255, 255, 255), x, y)
        self.render_text("normal", 18, (255, 255, 255, 255), x + 40, y)
        y_offset += offset

        self.render_text("◈", 32, (0, 255, 0, 255), x, y + y_offset)
        self.render_text("crosswalk", 18, (255, 255, 255, 255), x + 40, y + y_offset)
        y_offset += offset

        self.render_text("◈", 32, (0, 0, 255, 255), x, y + y_offset)
        self.render_text("intersection", 18, (255, 255, 255, 255), x + 40, y + y_offset)
        y_offset += offset

        self.render_text("◈", 32, (0, 165, 255, 255), x, y + y_offset)
        self.render_text("oneway", 18, (255, 255, 255, 255), x + 40, y + y_offset)
        y_offset += offset

        self.render_text("◈", 32, (130, 0, 75, 255), x, y + y_offset)
        self.render_text("highwayLeft", 18, (255, 255, 255, 255), x + 40, y + y_offset)
        y_offset += offset

        self.render_text("◈", 32, (193, 182, 255, 255), x, y + y_offset)
        self.render_text("highwayRight", 18, (255, 255, 255, 255), x + 40, y + y_offset)
        y_offset += offset

        self.render_text("◈", 32, (255, 255, 255, 255), x, y + y_offset)
        self.render_text("roundabout", 18, (255, 255, 255, 255), x + 40, y + y_offset)
        y_offset += offset

        y_offset = 60
        x_offset = 180
        self.render_text("◈", 32, (255, 255, 0, 255), x + x_offset, y + y_offset)
        self.render_text("stopline", 18, (255, 255, 255, 255), x + x_offset + 40, y + y_offset)
        y_offset += offset

        self.render_text("◈", 32, (180, 130, 70, 255), x + x_offset, y + y_offset)
        self.render_text("dotted", 18, (255, 255, 255, 255), x + x_offset + 40, y + y_offset)
        y_offset += offset

        self.render_text("◈", 32, (128, 0, 128, 255), x + x_offset, y + y_offset)
        self.render_text("dotted_crosswalk", 18, (255, 255, 255, 255), x + x_offset + 40, y + y_offset)
        y_offset += offset

    def draw_markers(self):
        for coord in self.cursor_coords:
            # x = coord[0] / MapData.REAL_WORLD_WIDTH.value * self.width()
            # y = coord[1] / MapData.REAL_WORLD_HEIGHT.value * self.height()
            # draw_marker(self.marker_vbo, self.circle_count, self.cross_count, x, y)
            pass

    def draw_detected_objects(self):
        if True:
            if self.detected_data is None or len(self.detected_data) == 0:
                return
            x = self.detected_data[0, self.road_msg_dict['x']]
            y = MapData.REAL_WORLD_HEIGHT.value - self.detected_data[0, self.road_msg_dict['y']]
            yaw = self.detected_data[0, self.road_msg_dict['orientation']]
            z = self.detected_data[0, self.road_msg_dict['z']]
            speed = self.detected_data[0, self.road_msg_dict['speed']]
            if self.waypoints is not None and not self.main_window.show_barca:
                # draw_path_node(self.waypoints, self.width(), self.height())
                pass
            self.main_window.car_widget.set_car_data(yaw / np.pi * 180, x, y, z)
            self.main_window.meter_widget.set_yaw(yaw / np.pi * 180)
            self.main_window.meter_widget.set_speed(speed * 100)
            for i in range(len(self.detected_data)):
                obj_type = self.detected_data[i, self.road_msg_dict['type']]
                x = self.detected_data[i, self.road_msg_dict['x']]
                y = self.detected_data[i, self.road_msg_dict['y']]
                orientation = self.detected_data[i, self.road_msg_dict['orientation']]

                # Convert map coordinates to pixel coordinates
                x = x / MapData.REAL_WORLD_WIDTH.value * self.width()
                y = y / MapData.REAL_WORLD_HEIGHT.value * self.height()
                # orientation = 2 * np.pi - orientation
                orientation = - orientation

                if self.object_dict[obj_type] == 'Car':
                    if i == 0:
                        self.renderer.draw_car(x, y, math.degrees(-orientation), 0.55, (1.0, 0.0, 0.0, 1.0))
                    else:
                        self.renderer.draw_car(x, y, math.degrees(-orientation), 0.55, (1.0, 0.0, 1.0, 1.0))
                else:
                    texture, vbo = self.sign_vbos[int(obj_type)]
                    self.renderer.draw_2D_texture(texture, vbo, x, y, 0.1)

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

    def cleanup_gl_resources(self):
        self.stop_drawing = True
        try:
            gl.glFlush()
        except Exception:
            pass

    def update_mouse_pos(self):
        if self.show_mouse:
            self.cursor_coords_label.show()
        else:
            self.cursor_coords_label.hide()
            return

        if self.current_mouse_pos is not None:
            widget_width = self.width()
            widget_height = self.height()
            if widget_height == 0 or widget_width == 0:
                return

            # Get mouse position in widget coordinates
            x_scene = self.current_mouse_pos.x()
            y_scene = self.current_mouse_pos.y()

            x_world, y_world = self.get_real_world_coords(x_scene, y_scene)
            # Update label text and position
            self.cursor_coords_label.setText(f"  ({x_world:.2f}, {y_world:.2f}) ")
            self.cursor_coords_label.move(
                int(x_scene - self.cursor_coords_label.width() / 2),
                int(y_scene - 60))

    def get_real_world_coords(self, x_scene, y_scene):
        widget_width = self.width()
        widget_height = self.height()
        if widget_height == 0 or widget_width == 0:
            return (0.0, 0.0)

        # Convert screen coordinates to NDC (Normalized Device Coordinates)
        ndc_x = (2.0 * x_scene / widget_width) - 1.0
        ndc_y = 1.0 - (2.0 * y_scene / widget_height)

        # Compute half width and height in view space after zoom
        hw = (widget_width / self.view_zoom) / 2.0
        hh = (widget_height / self.view_zoom) / 2.0

        # Apply inverse projection to get view space coordinates
        x_view = ndc_x * hw
        y_view = ndc_y * hh

        # Apply inverse view matrix to get OpenGL world coordinates
        world_x = x_view + self.view_center.x
        world_y = y_view + self.view_center.y

        # Convert OpenGL coordinates to real-world system
        real_world_x = (world_x + (widget_width / 2)) * (MapData.REAL_WORLD_WIDTH.value / widget_width)
        real_world_y = (world_y + (widget_height / 2)) * (MapData.REAL_WORLD_HEIGHT.value / widget_height)

        return (real_world_x, real_world_y)

    def update_waypoints(self):
        self.waypoints_renderer.update_waypoints(self.state_refs_np, self.attributes_np, self.width(), self.height())

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
        self.run_statistics.move(
            w - self.run_statistics.width() - 5,
            5
        )
        self.waypoints_renderer.update_waypoints(self.state_refs_np, self.attributes_np, self.width(), self.height())

    def mouseDoubleClickEvent(self, event):
        x_world, y_world = self.get_real_world_coords(event.pos().x(), event.pos().y())
        self.cursor_coords.append((x_world, y_world))
        self.cursor_x = x_world
        self.cursor_y = y_world

    def mousePressEvent(self, event):
        if event.button() == QtCore.Qt.LeftButton:
            self.drag_start = event.pos()
            self.base_view_center = glm.vec2(self.view_center)
            self.last_mouse_pos = event.pos()

    def mouseMoveEvent(self, event):
        self.current_mouse_pos = event.pos()
        if event.buttons() & QtCore.Qt.LeftButton and self.drag_start is not None:
            # Calculate delta movement
            delta = event.pos() - self.last_mouse_pos
            self.last_mouse_pos = event.pos()

            # Convert pixel delta to world coordinates
            zoom_factor = 1.0 / self.view_zoom
            half_width = self.width() * zoom_factor / 2
            half_height = self.height() * zoom_factor / 2

            # Calculate world units per pixel
            world_per_pixel_x = (2 * half_width) / self.width()
            world_per_pixel_y = (2 * half_height) / self.height()

            # Update view center
            self.view_center.x -= delta.x() * world_per_pixel_x
            self.view_center.y += delta.y() * world_per_pixel_y

    def wheelEvent(self, event):
        # Get mouse position in normalized device coordinates
        mouse_pos = event.pos()
        mouse_x = 2.0 * mouse_pos.x() / self.width() - 1.0
        mouse_y = 1.0 - 2.0 * mouse_pos.y() / self.height()

        # Store pre-zoom values
        old_zoom = self.view_zoom
        zoom_factor = 1.15 if event.angleDelta().y() > 0 else 0.85
        self.view_zoom *= zoom_factor

        # Keep zoom within bounds
        self.view_zoom = max(1.0, min(5.0, self.view_zoom))

        # Calculate new center to maintain mouse position
        zoom_ratio = old_zoom / self.view_zoom
        self.view_center += glm.vec2(
            mouse_x * (self.width() / 2) * (1 - zoom_ratio) / old_zoom,
            mouse_y * (self.height() / 2) * (1 - zoom_ratio) / old_zoom
        )

    ##################
    # Callbacks
    ##################

    def update_params(self, req) -> None:
        try:
            max_retries = 50
            retries = 0
            params = self.server.utility_node_client.params
            while (retries < max_retries):
                if (len(params.state_refs) > 0 and len(params.attributes) > 0):
                    self.state_refs_np = params.state_refs.popleft()
                    self.attributes_np = params.attributes.popleft()
                    print("state ref shape: ", self.state_refs_np.shape)
                    # print first 3 rows
                    print("state ref: ", self.state_refs_np.T[:, :3])
                    path = os.path.dirname(os.path.abspath(__file__))
                    np.savetxt(os.path.join(path, 'state_refs.txt'), self.state_refs_np.T, fmt='%.4f')
                    print("saved state refs")
                    return TriggerResponse(success=True, message="Parameters updated")
                retries += 1
                time.sleep(0.1)
            print("Failed to update params: timeout")
            return TriggerResponse(success=False, message="Failed to update: timeout")
        except Exception as e:
            print(f"Failed to update parameters: {e}")
            return TriggerResponse(success=False, message=f"Failed to update: {e}")

    def road_objects_callback(self, road_object) -> None:
        self.detected_data = np.array(road_object.data).reshape(-1, self.road_msg_length)

    def waypoint_callback(self, waypoints) -> None:
        self.waypoints = waypoints.data

    def sign_callback(self, sign) -> None:
        if sign.data:
            self.numObj = len(sign.data) // 10
            if self.numObj > 0:
                self.detected_objects = np.array(sign.data)  # .reshape(-1, 7).T
        else:
            self.numObj = 0

    def call_waypoint_service(self, run):
        try:
            self.server.utility_node_client.send_waypoints_srv(run.vref_name, run.path_name, run.x_init, run.y_init, run.yaw_init)
            max_retries = 50
            retries = 0
            res = self.server.utility_node_client.waypoints_srv_msg
            while (retries < max_retries):
                if (len(res.state_refs.data) > 0 and len(res.wp_attributes.data) > 0):
                    self.state_refs_np = np.array(res.state_refs.data).reshape(-1, 3).T
                    self.attributes_np = np.array(res.wp_attributes.data)
                    print("Waypoints service call successful. shape: ", self.state_refs_np.shape)
                    self.main_window.run_overlay.set_run_name(run.path_name)
                    self.run_statistics.set_total_path_distance()
                    if self.main_window.show_barca:
                        self.main_window.barca_widget.waypoints_renderer.update_waypoints(self.state_refs_np)
                    else:
                        self.waypoints_renderer.update_waypoints(self.state_refs_np, self.attributes_np, self.width(), self.height())
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to send waypoints service call")
        except Exception as e:
            raise e
