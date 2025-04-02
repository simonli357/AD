from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from std_srvs.srv import TriggerResponse
from OpenGL import GL as gl
from .view import HidableOverlay
from .opengl.vbos import track_vbo, circle_vbo, sign_vbo, marker_vbo
from .opengl.renderer import draw_track, draw_destination, draw_car, draw_lane, draw_sign, draw_waypoint, draw_path_node, draw_marker
from ..opengl.loaders import load_obj, load_texture
from ..enums import MapData

import pandas as pd
import os
import time
import numpy as np


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
        self.pan_x = 0
        self.pan_y = 0
        self.init_zoom = 1.2
        self.max_zoom = 1.0
        self.zoom_level = self.max_zoom
        self.last_mouse_pos = None
        self.current_mouse_pos = None
        self.show_mouse = True

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
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glEnable(gl.GL_BLEND)
        gl.glBlendFunc(gl.GL_SRC_ALPHA, gl.GL_ONE_MINUS_SRC_ALPHA)

        self.track_texture = load_texture(self.track_model_path)
        self.track_vbo = track_vbo(self.width(), self.height())
        self.car_model = load_obj(self.car_model_path)
        self.dest_vbo = circle_vbo(7, 25)
        self.wp_vbo = circle_vbo(2, 10)
        self.path_node_vbo = circle_vbo(1, 3)
        self.marker_vbo, (self.circle_count, self.cross_count) = marker_vbo()

        self.sign_vbos = []

        for path in self.sign_images:
            texture = load_texture(path)
            vbo = sign_vbo()
            self.sign_vbos.append((texture, vbo))

    def paintGL(self):
        if self.stop_drawing:
            return

        curr_widget_width = self.width()
        curr_widget_height = self.height()

        self.update_mouse_pos()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)
        gl.glPushAttrib(gl.GL_ALL_ATTRIB_BITS)

        # Set up projection matrix
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()

        gl.glOrtho(0.0, curr_widget_width, 0.0, curr_widget_height, -20, 20)

        # Apply cursor zoom / translation
        gl.glTranslatef(self.pan_x, self.pan_y, 0)
        gl.glScalef(1 / self.zoom_level, 1 / self.zoom_level, 1 / self.zoom_level)

        # Draw track
        draw_track(self.track_texture, self.track_vbo, 4)

        # Draw objects
        self.illustrate_path()
        if self.show_gt:
            for index, row in self.data.iterrows():
                entity_type, orientation = row['Type'], row['Orientation']

                x = row['X'] / MapData.REAL_WORLD_WIDTH.value * self.width()
                y = row['Y'] / MapData.REAL_WORLD_HEIGHT.value * self.height()

                # orientation = 2 * np.pi - orientation
                orientation = - orientation

                if entity_type == 'Intersection':
                    if self.show_signs:
                        # self.draw_intersection(image, pixel_x, pixel_y, orientation, 20)
                        pass
                elif entity_type == 'Lane':
                    if self.show_lanes:
                        draw_lane(x, y, self.width(), self.height(), orientation)
                elif entity_type == 'Car':
                    if self.show_cars:
                        draw_car(x, y, orientation, self.car_model, (1.0, 1.0, 0.0, 1.0))
                elif entity_type == 'Destination':
                    if self.show_destinations:
                        draw_destination(self.dest_vbo, x, y)
                else:
                    if self.show_signs:
                        sign_index = self.get_key_from_value(entity_type)
                        texture, vbo = self.sign_vbos[sign_index]
                        draw_sign(x, y, texture, vbo)

        self.draw_detected_objects()
        self.draw_markers()
        
        gl.glPopAttrib()

        if self.zoom_level == 1.0:
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
            x = coord[0] / MapData.REAL_WORLD_WIDTH.value * self.width()
            y = coord[1] / MapData.REAL_WORLD_HEIGHT.value * self.height()
            draw_marker(self.marker_vbo, self.circle_count, self.cross_count, x, y)

    def draw_detected_objects(self):
        if True:
            if self.waypoints is not None and not self.main_window.show_barca:
                for i in range(0, len(self.waypoints) - 1, 8):
                    x = self.waypoints[i] / MapData.REAL_WORLD_WIDTH.value * self.width()
                    y = self.waypoints[i + 1] / MapData.REAL_WORLD_HEIGHT.value * self.height()
                    draw_path_node(self.path_node_vbo, x, y)
            if self.detected_data is None or len(self.detected_data) == 0:
                return
            x = self.detected_data[0, self.road_msg_dict['x']]
            y = MapData.REAL_WORLD_HEIGHT.value - self.detected_data[0, self.road_msg_dict['y']]
            yaw = self.detected_data[0, self.road_msg_dict['orientation']]
            z = self.detected_data[0, self.road_msg_dict['z']]
            speed = self.detected_data[0, self.road_msg_dict['speed']]
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
                        draw_car(x, y, orientation, self.car_model, (1.0, 0.0, 0.0, 1.0))
                    else:
                        draw_car(x, y, orientation, self.car_model, (1.0, 0.0, 1.0, 1.0))
                else:
                    texture, vbo = self.sign_vbos[int(obj_type)]
                    draw_sign(x, y, texture, vbo)

    def illustrate_path(self):
        if self.state_refs_np is None or self.attributes_np is None or not self.show_path:
            return

        ATTRIBUTES = {
            "normal": (0, 255, 255),        # Yellow
            "crosswalk": (0, 255, 0),         # Green
            "intersection": (0, 0, 255),  # Red
            "oneway": (0, 165, 255),          # Orange
            "highwayLeft": (130, 0, 75),      # Indigo
            "highwayRight": (193, 182, 255),         # Light pink
            "roundabout": (255, 255, 255),    # White
            "stopline": (255, 255, 0),    # Cyan
            "dotted": (180, 130, 70),         # Steel blue
            "dotted_crosswalk": (128, 0, 128),    # Purple
        }

        for i in range(0, self.state_refs_np.shape[1], 8):
            color = (0, 255, 255)
            attr = self.attributes_np[i]

            # Assign colors based on attributes
            if attr == 0 or attr == 100:
                color = ATTRIBUTES["normal"]
            elif attr == 1 or attr == 101:
                color = ATTRIBUTES["crosswalk"]
            elif attr == 2 or attr == 102:
                color = ATTRIBUTES["intersection"]
            elif attr == 3 or attr == 103:
                color = ATTRIBUTES["oneway"]
            elif attr == 4 or attr == 104:
                color = ATTRIBUTES["highwayLeft"]
            elif attr == 5 or attr == 105:
                color = ATTRIBUTES["highwayRight"]
            elif attr == 6 or attr == 106:
                color = ATTRIBUTES["roundabout"]
            elif attr == 7 or attr == 107:
                color = ATTRIBUTES["stopline"]
            elif attr == 8 or attr == 108:
                color = ATTRIBUTES["dotted"]
            elif attr == 9 or attr == 109:
                color = ATTRIBUTES["dotted_crosswalk"]

            # if attr == 7 or attr == 107:
            #     color = ATTRIBUTES["stopline"]
            # else:
            #     color = (0, 0, 0)

            x = self.state_refs_np[0, i] / MapData.REAL_WORLD_WIDTH.value * self.width()
            y = self.state_refs_np[1, i] / MapData.REAL_WORLD_HEIGHT.value * self.height()

            gl_color = (color[2], color[1], color[0])

            draw_waypoint(self.wp_vbo, x, y, gl_color)

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

            # Convert to OpenGL projection space (y=0 at bottom)
            y_gl_proj = widget_height - y_scene

            # Apply inverse transformations for pan/zoom
            x_original = (x_scene - self.pan_x) * self.zoom_level
            y_original = (y_gl_proj - self.pan_y) * self.zoom_level

            # Convert to real-world coordinates
            try:
                x_world = (x_original / widget_width) * MapData.REAL_WORLD_WIDTH.value
                y_world = (y_original / widget_height) * MapData.REAL_WORLD_HEIGHT.value
            except ZeroDivisionError:
                return

            # Update label text and position
            self.cursor_coords_label.setText(f"  ({x_world:.2f}, {y_world:.2f}) ")
            self.cursor_coords_label.move(
                int(x_scene - self.cursor_coords_label.width() / 2),
                int(y_scene - 60))

    def get_real_world_coords(self, x_scene, y_scene):
        widget_width = self.width()
        widget_height = self.height()
        if widget_height == 0 or widget_width == 0:
            return

        # Convert to OpenGL projection space (y=0 at bottom)
        y_gl_proj = widget_height - y_scene

        # Apply inverse transformations for pan/zoom
        x_original = (x_scene - self.pan_x) * self.zoom_level
        y_original = (y_gl_proj - self.pan_y) * self.zoom_level

        # Convert to real-world coordinates
        try:
            x_world = (x_original / widget_width) * MapData.REAL_WORLD_WIDTH.value
            y_world = (y_original / widget_height) * MapData.REAL_WORLD_HEIGHT.value
        except ZeroDivisionError:
            return

        return x_world, y_world

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
        self.track_vbo.delete()
        self.track_vbo = track_vbo(w, h)
        self.update_mouse_pos()
        self.run_statistics.move(
            w - self.run_statistics.width() - 5,
            5
        )

    def mousePressEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            self.last_mouse_pos = event.pos()
        if event.buttons() == QtCore.Qt.RightButton:
            self.cursor_coords.clear()

    def mouseDoubleClickEvent(self, event):
        x_world, y_world = self.get_real_world_coords(event.pos().x(), event.pos().y())
        self.cursor_coords.append((x_world, y_world))
        self.cursor_x = x_world
        self.cursor_y = y_world

    def mouseMoveEvent(self, event):
        self.current_mouse_pos = event.pos()
        if event.buttons() == QtCore.Qt.LeftButton and self.last_mouse_pos is not None:
            if self.zoom_level >= self.max_zoom:
                return

            sensitivity = 1.0

            dx = event.pos().x() - self.last_mouse_pos.x()
            dy = event.pos().y() - self.last_mouse_pos.y()
            self.last_mouse_pos = event.pos()

            widget_height = self.height()
            if widget_height == 0:
                widget_height = 1

            max_allowed_x = self.width() * (1 / self.zoom_level - 1)
            max_allowed_y = self.height() * (1 / self.zoom_level - 1)

            new_pan_x = self.pan_x + dx * sensitivity
            new_pan_y = self.pan_y - dy * sensitivity

            new_pan_x = max(-max_allowed_x, min(0, new_pan_x))
            new_pan_y = max(-max_allowed_y, min(0, new_pan_y))

            self.pan_x = new_pan_x
            self.pan_y = new_pan_y

    def wheelEvent(self, event):
        delta = -event.angleDelta().y()
        if delta != 0:
            widget_width = self.width()
            widget_height = self.height()
            if widget_width <= 0 or widget_height <= 0:
                return

            # Calculate zoom parameters
            zoom_factor = 1.15 if delta > 0 else 0.85
            new_zoom = max(0.3, min(self.max_zoom, self.zoom_level * zoom_factor))

            if new_zoom == self.zoom_level:
                return

            self.zoom_level = new_zoom

            # Force center at max zoom
            if self.zoom_level == self.max_zoom:
                self.pan_x = 0
                self.pan_y = 0

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
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to send waypoints service call")
        except Exception as e:
            raise e
