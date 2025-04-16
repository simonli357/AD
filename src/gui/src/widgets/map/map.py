from PyQt5 import QtWidgets, QtCore
from std_srvs.srv import TriggerResponse
from OpenGL import GL as gl
from .view import HidableOverlay
from ..opengl.shader import ShaderRenderer
from ..opengl.waypoints import WaypointsRenderer
from ..opengl.destinations import DestinationsRenderer
from ..opengl.loaders import load_2D_texture
from ..enums import MapData, NamedColor, OpenGLContextName

import pandas as pd
import os
import time
import numpy as np
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

        self.road_msg_length = 8
        self.road_msg_dict = {
            'type': 0,
            'x': 1,
            'y': 2,
            'orientation': 3,
            'speed': 4,
            'confidence': 5,
            'z': 6,
            'id': 7
        }

        self.detected_data = None
        self.waypoints = None
        self.numObj = 0
        self.detected_objects = np.zeros(7)

        current_dir = os.path.dirname(os.path.abspath(__file__))
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

        self.car_yaw = 0

        self.sign_images = []
        self.sign_images.append(os.path.join(self.assets_dir, 'oneway.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'highway_entrance.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'stopsign.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'roundabout.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'parking.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'crosswalk.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'noentry.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'highway_exit.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'priority.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'trafficlight.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'roadblock.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'pedestrian.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'car.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'trafficlight_green.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'trafficlight_yellow.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'trafficlight_red.png'))
        self.sign_images.append(os.path.join(self.assets_dir, 'stopsign2.png'))

        self.setup_ui()

    def setup_ui(self):
        self.run_statistics = HidableOverlay(self)

    def get_key_from_value(self, value):
        return self.reverse_object_dict.get(value, None)

    def render_widget(self) -> None:
        self.update()

    def set_steer(self, steer):
        self.run_statistics.set_car_rotation(self.car_yaw, steer)

    def initializeGL(self):
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glDepthFunc(gl.GL_LEQUAL)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)
        gl.glDisable(gl.GL_LINE_SMOOTH)    # Avoid anti-aliasing overhead
        gl.glDisable(gl.GL_POLYGON_SMOOTH)
        gl.glDisable(gl.GL_MULTISAMPLE)    # Disable MSAA if not used
        gl.glPolygonMode(gl.GL_FRONT_AND_BACK, gl.GL_FILL)  # Fastest mode
        gl.glShadeModel(gl.GL_FLAT)        # Faster than GL_SMOOTH if applicable

        self.ortho_proj_mat = glm.ortho(0.0, self.width(), self.height(), 0.0, -1.0, 1.0)
        self.ortho_view_mat = glm.mat4(1.0)

        self.waypoints_renderer = WaypointsRenderer(track='bfmc')
        self.destinations_renderer = DestinationsRenderer()
        self.shader_renderer = ShaderRenderer(ctx_name=OpenGLContextName.MAP)

        self.update_destinations()

        self.sign_models = []
        for path in self.sign_images:
            texture = load_2D_texture(path)
            self.sign_models.append(texture)

    def paintGL(self):
        if self.stop_drawing:
            return
        if self.view_zoom == 1:
            self.view_center = glm.vec2(0, 0)

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        zoom_factor = 1.0 / self.view_zoom
        half_width = self.width() * zoom_factor / 2
        half_height = self.height() * zoom_factor / 2

        self.proj_mat = glm.ortho(
            -half_width, half_width,
            -half_height, half_height,
            -100.0, 100.0
        )

        self.view_mat = glm.lookAt(
            glm.vec3(self.view_center.x, self.view_center.y, 1.0),
            glm.vec3(self.view_center.x, self.view_center.y, 0.0),
            glm.vec3(0.0, 1.0, 0.0)
        )

        self.shader_renderer.draw_texture(
            mat=self.shader_renderer.bfmc_track_model,
            x=-self.width() / 2,
            y=-self.height() / 2,
            z=0,
            scale=(self.width(), self.height()),
            view_matrix=self.view_mat,
            proj_matrix=self.proj_mat
        )

        if self.show_path:
            self.waypoints_renderer.draw(self.proj_mat, self.view_mat)

        self.draw_gt()
        self.draw_markers()
        self.draw_detected_objects()
        self.draw_path_nodes()

        if self.view_zoom == 1.0:
            self.draw_legend(self.width() / 2.7, self.height() / 3)
            pass

        self.update_mouse_pos()

    def draw_gt(self):
        if not self.show_gt:
            return

        if self.show_destinations:
            self.destinations_renderer.draw((0.0, 0.7, 0.7, 1.0), self.proj_mat, self.view_mat)

            for x, y in self.run_statistics.visited:
                self.shader_renderer.draw_circle(self.get_gl_coords(x, y), 0.7, (0.0, 1.0, 0.0), self.view_mat, self.proj_mat)

        for index, row in self.data.iterrows():
            entity_type, orientation = row['Type'], row['Orientation']

            x, y = self.get_gl_coords(row['X'], row['Y'])

            # orientation = 2 * np.pi - orientation
            orientation = - orientation

            if entity_type == 'Intersection':
                if self.show_signs:
                    # self.draw_intersection(image, pixel_x, pixel_y, orientation, 20)
                    continue
            elif entity_type == 'Lane':
                if self.show_lanes:
                    self.draw_lane(x, y, orientation)
            elif entity_type == 'Car':
                if self.show_cars:
                    self.shader_renderer.draw_car(x, y, -orientation, NamedColor.RED, 0.55, self.view_mat, self.proj_mat)
                    self.shader_renderer.draw_axis2D(x, y, -orientation, 25, self.view_mat, self.proj_mat)
            else:
                if self.show_signs:
                    sign_index = self.get_key_from_value(entity_type)
                    if sign_index is not None:
                        mat = self.sign_models[sign_index]
                        self.shader_renderer.draw_texture(mat, x, y, 0.05, (20, 20), self.view_mat, self.proj_mat)

    def draw_lane(self, x, y, orientation):
        """Draw lane markings using OpenGL lines"""
        # Normalize orientation within 0-2π
        orientation %= 2 * np.pi
        x_max, y_max = self.get_gl_coords(MapData.REAL_WORLD_WIDTH.value, MapData.REAL_WORLD_HEIGHT.value)

        # Determine lane direction and color
        if abs(orientation) < 0.1 or abs(orientation - 2 * np.pi) < 0.1:
            # Horizontal lane (east-west)
            color = (0.0, 1.0, 0.0, 1.0)  # Green
            start = (x_max, y)
            end = (-x_max, y)
            self.shader_renderer.draw_line(start, end, color, self.view_mat, self.proj_mat)
            return
        elif abs(orientation - np.pi) < 0.1:
            # Horizontal lane (west-east)
            color = (1.0, 0.0, 0.0, 1.0)  # Red
            start = (-x_max, y)
            end = (x_max, y)
            self.shader_renderer.draw_line(start, end, color, self.view_mat, self.proj_mat)
            return
        elif abs(orientation - np.pi / 2) < 0.1:
            # Vertical lane (north-south)
            color = (0.0, 0.0, 1.0, 1.0)  # Blue
            start = (x, y_max)
            end = (x, -y_max)
            self.shader_renderer.draw_line(start, end, color, self.view_mat, self.proj_mat)
            return
        elif abs(orientation - 3 * np.pi / 2) < 0.1:
            # Vertical lane (south-north)
            color = (1.0, 1.0, 0.0, 1.0)  # Yellow
            start = (x, y_max)
            end = (x, -y_max)
            self.shader_renderer.draw_line(start, end, color, self.view_mat, self.proj_mat)
            return

    def draw_legend(self, x, y):
        viewport = gl.glGetIntegerv(gl.GL_VIEWPORT)
        screen_width = viewport[2]
        screen_height = viewport[3]
        x, y = 0.37 * screen_width, 0.35 * screen_height
        height = 5

        offset = 35
        y_offset = 0
        self.shader_renderer.draw_triangle(x, y, 0, np.radians(180), (24.0, 24.0), NamedColor.YELLOW.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("NORMAL", x + 62, y - height, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.GREEN.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("CROSSWALK", x + 80, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.RED.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("INTERSECTION", x + 88, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.ORANGE.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("ONEWAY", x + 64, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.INDIGO.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("HIGHWAY LEFT", x + 90, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.LIGHT_PINK.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("HIGHWAY RIGHT", x + 94, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        y_offset = 0
        x_offset = 200
        self.shader_renderer.draw_triangle(x + x_offset, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.WHITE.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("ROUNDABOUT", x + x_offset + 84, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x + x_offset, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.CYAN.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("STOPLINE", x + x_offset + 66, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x + x_offset, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.STEEL_BLUE.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("DOTTED", x + x_offset + 60, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

        self.shader_renderer.draw_triangle(x + x_offset, y + y_offset, 0, np.radians(180), (24.0, 24.0), NamedColor.PURPLE.value, self.ortho_view_mat, self.ortho_proj_mat)
        self.shader_renderer.text_renderer.render_text("DOTTED CROSSWALK", x + x_offset + 116, y - height + y_offset, 1.0, (1, 1, 1), self.ortho_proj_mat)
        y_offset += offset

    def draw_markers(self):
        for coord in self.cursor_coords:
            x, y = self.get_gl_coords(coord[0], coord[1])
            self.shader_renderer.draw_marker(x, y, (1, 0, 0, 1), 10.0, view_matrix=self.view_mat, proj_matrix=self.proj_mat)

    def draw_detected_objects(self):
        if self.detected_data is None or len(self.detected_data) == 0:
            return
        car_x = self.detected_data[0, self.road_msg_dict['x']]
        car_y = MapData.REAL_WORLD_HEIGHT.value - self.detected_data[0, self.road_msg_dict['y']]
        car_yaw = self.detected_data[0, self.road_msg_dict['orientation']]
        car_z = self.detected_data[0, self.road_msg_dict['z']]
        car_speed = self.detected_data[0, self.road_msg_dict['speed']]
        self.main_window.car_widget.set_car_data(car_yaw / np.pi * 180, car_speed, car_x, car_y, car_z)
        self.run_statistics.set_car_pose(car_x, car_y, car_z)
        self.car_yaw = car_yaw
        for i in range(len(self.detected_data)):
            obj_type = self.detected_data[i, self.road_msg_dict['type']]
            x_real = self.detected_data[i, self.road_msg_dict['x']]
            y_real = self.detected_data[i, self.road_msg_dict['y']]
            orientation = self.detected_data[i, self.road_msg_dict['orientation']]

            # Convert map coordinates to pixel coordinates
            x, y = self.get_gl_coords(x_real, y_real)
            # orientation = 2 * np.pi - orientation
            orientation = - orientation

            if self.object_dict[obj_type] == 'Car':
                if i == 0:
                    self.shader_renderer.draw_car(x, y, -orientation, NamedColor.WHITE, 0.55, self.view_mat, self.proj_mat)
                    self.shader_renderer.draw_axis2D(x, y, -orientation, 25.0, self.view_mat, self.proj_mat)
                else:
                    self.shader_renderer.draw_car(x, y, -orientation, NamedColor.ORANGE, 0.55, self.view_mat, self.proj_mat)
                    self.shader_renderer.draw_axis2D(x, y, -orientation, 25.0, self.view_mat, self.proj_mat)
            else:
                texture = self.sign_models[int(obj_type)]
                self.shader_renderer.draw_texture(texture, x, y, 0, (20, 20), self.view_mat, self.proj_mat)
                self.shader_renderer.draw_axis2D(x, y, -orientation, 25.0, self.view_mat, self.proj_mat)

    def draw_path_nodes(self):
        if self.waypoints is None or len(self.waypoints) < 2:
            return
        x1, y1 = self.get_gl_coords(self.waypoints[0], self.waypoints[1])
        angle = 0
        for i in range(0, len(self.waypoints) - 1, 4):
            if i + 3 > len(self.waypoints):
                self.shader_renderer.draw_triangle(x1, y1, 2.0, angle, (4, 4), (1.0, 1.0, 0.0, 1.0), self.view_mat, self.proj_mat)
            else:
                x2, y2 = self.get_gl_coords(self.waypoints[i + 2], self.waypoints[i + 3])
                dx = x2 - x1
                dy = y2 - y1
                angle = np.arctan2(dy, dx + (1e-5)) - np.pi / 2
                self.shader_renderer.draw_triangle(x1, y1, 2.0, angle, (4, 4), (1.0, 1.0, 0.0, 1.0), self.view_mat, self.proj_mat)
                x1, y1 = x2, y2

    def cleanup_gl_resources(self):
        self.stop_drawing = True
        try:
            gl.glFlush()
        except Exception:
            pass

    def update_mouse_pos(self):
        if not self.show_mouse:
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

            self.shader_renderer.text_renderer.render_text(f"X {x_world:.2f}   Y {y_world:.2f}", x_scene, y_scene - 30, 1.0, (0, 1, 0), self.ortho_proj_mat)

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

        return real_world_x, real_world_y

    def get_gl_coords(self, real_x, real_y):
        widget_width = self.width()
        widget_height = self.height()
        if widget_height == 0 or widget_width == 0:
            return (0.0, 0.0)

        # Convert real-world to OpenGL world coordinates
        world_x = (real_x * widget_width / MapData.REAL_WORLD_WIDTH.value) - (widget_width / 2)
        world_y = (real_y * widget_height / MapData.REAL_WORLD_HEIGHT.value) - (widget_height / 2)

        return world_x, world_y

    def update_waypoints(self):
        if hasattr(self, 'proj_mat') and hasattr(self, 'view_mat'):
            self.waypoints_renderer.update_waypoints(self.state_refs_np, self.attributes_np, self.width(), self.height())

    def update_destinations(self):
        self.destinations_renderer.update_data(self.data.iterrows(), self.width(), self.height())

    def update_visited_destination(self, x_visited, y_visited):
        self.main_window.car_widget.update_visited_destination(x_visited, y_visited)

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
        self.ortho_proj_mat = glm.ortho(0.0, w, h, 0.0, -1.0, 1.0)
        self.run_statistics.move(
            w - self.run_statistics.width() - 5,
            5
        )
        self.update_waypoints()
        self.update_destinations()

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
        if event.button() == QtCore.Qt.RightButton:
            self.cursor_coords.clear()

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
                    self.main_window.barca_widget.update_waypoints()
                    self.update_waypoints()
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to send waypoints service call")
        except Exception as e:
            raise e
