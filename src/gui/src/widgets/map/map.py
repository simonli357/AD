from PyQt5 import QtWidgets, QtCore
from PyQt5.Qt import QPainter, QFont, QColor
from std_srvs.srv import TriggerResponse
from OpenGL import GL as gl
from .opengl.vbos import track_vbo
from .opengl.renderer import draw_track
from ..utils.opengl import qt_save_gl_state, qt_restore_gl_state, load_obj, load_texture

import pandas as pd
import os
import cv2
import time
import numpy as np


class MapWidget(QtWidgets.QOpenGLWidget):
    update_map_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.server = self.main_window.server
        self.markers = []
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

        self.cars_drawn = False

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

        self.sign_images = []
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'oneway.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'highway_entrance.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'stopsign.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'roundabout.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'parking.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'crosswalk.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'noentry.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'highway_exit.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'priority.png')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'trafficlight.png')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'roadblock.png')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'pedestrian.png')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'car.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'trafficlight_green.png')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'trafficlight_yellow.png')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'trafficlight_red.png')))
        self.sign_images.append(cv2.imread(os.path.join(self.assets_dir, 'stopsign.jpg')))

        self.car_icon_path = os.path.join(self.assets_dir, 'car_top.png')

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

        fmt = self.format()
        fmt.setAlphaBufferSize(8)  # Enable alpha channel
        self.setFormat(fmt)

        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.track_model_path = os.path.join(current_dir, 'assets', 'track.png')
        self.car_model_path = os.path.join(current_dir, 'assets', 'car.obj')

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

    def get_key_from_value(self, value):
        return self.reverse_object_dict.get(value, None)

    def render_widget(self) -> None:
        self.update()

    def initializeGL(self):
        gl.glEnable(gl.GL_DEPTH_TEST)
        gl.glClearColor(0.0, 0.0, 0.0, 1.0)

        self.track_texture = load_texture(self.track_model_path)
        self.track_vbo = track_vbo(self.width(), self.height())
        self.car_model = load_obj(self.car_model_path)

    def paintGL(self):
        if self.stop_drawing:
            return

        curr_widget_width = self.width()
        curr_widget_height = self.height()

        self.update_mouse_pos()

        qt_save_gl_state()

        gl.glClear(gl.GL_COLOR_BUFFER_BIT | gl.GL_DEPTH_BUFFER_BIT)

        # Set up projection matrix
        gl.glMatrixMode(gl.GL_PROJECTION)
        gl.glLoadIdentity()

        gl.glOrtho(
            0.0,
            curr_widget_width,
            0.0,
            curr_widget_height,
            -1,
            1
        )

        # Apply cursor zoom / translation
        gl.glTranslatef(self.pan_x, self.pan_y, 0)
        gl.glScalef(1 / self.zoom_level, 1 / self.zoom_level, 1 / self.zoom_level)

        # Global Transforms
        draw_track(self.track_texture, self.track_vbo, 4)

        qt_restore_gl_state()

    def render_text(self, text, size, color: (int, int, int, int), x, y) -> None:
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

            # Convert to normalized device coordinates [-1, 1]
            x_ndc = 2 * (x_scene / widget_width) - 1
            y_ndc = (1 - 2 * (y_scene / widget_height))

            # Calculate orthographic projection bounds
            aspect = widget_width / widget_height
            half_zoom = self.zoom_level * 0.5
            left = self.pan_x - half_zoom * aspect
            right = self.pan_x + half_zoom * aspect
            bottom = self.pan_y - half_zoom
            top = self.pan_y + half_zoom

            # Convert to world coordinates
            x_world = left + (x_ndc + 1) * (right - left) / 2
            y_world = bottom + (y_ndc + 1) * (top - bottom) / 2

            self.cursor_coords_label.setText(f"  ({x_world:.2f}, {y_world:.2f}) ")
            self.cursor_coords_label.move(
                int(x_scene - self.cursor_coords_label.width() / 2),
                int(y_scene - 60))

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
        self.update_mouse_pos()

    def mousePressEvent(self, event):
        if event.buttons() == QtCore.Qt.LeftButton:
            self.last_mouse_pos = event.pos()

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

            self.update()

    def wheelEvent(self, event):
        delta = event.angleDelta().y()
        if delta != 0:
            sensitivity = 0.0005
            new_zoom = self.zoom_level - delta * sensitivity
            new_zoom = max(0.1, min(self.max_zoom, new_zoom))
            if new_zoom != self.zoom_level:
                self.zoom_level = new_zoom
                # Reset pan when returning to initial zoom
                if self.zoom_level == self.max_zoom:
                    self.pan_x = 0
                    self.pan_y = 0
                self.update()

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
                    self.update_map_display()
                    self.graphics_view.set_total_path_distance()
                    self.main_window.run_overlay.set_run_name(run.path_name)
                    self.main_window.barca_widget.waypoints_renderer.update_waypoints(self.state_refs_np)
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to send waypoints service call")
        except Exception as e:
            raise e
