from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QGraphicsView, QSizePolicy, QLabel
from std_srvs.srv import TriggerResponse
import pandas as pd

import os
import cv2
import time
import numpy as np
import math


class MapWidget(QtWidgets.QWidget):
    update_map_signal = QtCore.pyqtSignal()

    def __init__(self, parent=None):
        super().__init__(parent)
        self.main_window = self.parent()
        self.server = self.main_window.server
        self.current_zoom = 1.0
        self.min_zoom = 1.0
        self.markers = []
        self.cursor_coords = []
        self.cursor_x = 3.86
        self.cursor_y = 3.62

        self.position_label = QLabel('Position: (x: 0.0, y: 0.0, yaw: 0.0, z: 0.0)')
        self.cursor_label = QLabel('Cursor: (x: 0.0, y: 0.0, yaw: 0.0)')
        self.speed_label = QLabel('Speed: 0.0 m/s')

        self.show_signs = False
        self.show_lanes = False
        self.show_cars = False
        self.show_destinations = True
        self.show_path = True
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

        current_dir = os.path.dirname(os.path.abspath(__file__))
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

        self.setup_ui()

        # Map configuration
        self.scale_factor = 1.25
        self.image_width_real = 20.696
        self.image_height_real = 13.786
        self.image_width = int(800 * self.scale_factor)
        self.image_height = int(533 * self.scale_factor)

        # Initialize map image
        self.load_map_image()

        # Timer for updates
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.update_map_display)
        self.timer.start(100)

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setContentsMargins(0, 0, 0, 0)

        # Graphics View setup
        self.graphics_view = CustomGraphicsView(self)
        self.layout.addWidget(self.graphics_view)
        self.graphics_view.installEventFilter(self)

        # Scene setup
        self.scene = QtWidgets.QGraphicsScene(self)
        self.graphics_view.setScene(self.scene)
        self.map_item = QtWidgets.QGraphicsPixmapItem()
        self.scene.addItem(self.map_item)

    def load_map_image(self) -> None:
        map_image = cv2.imread(os.path.join(self.assets_dir, 'map1.png'))
        self.map_image = cv2.resize(map_image, (self.image_width, self.image_height))
        self.update_map_display()

    def update_map_display(self) -> None:
        if hasattr(self, 'map_image'):
            display_image = self.map_image.copy()

            if self.show_path:
                display_image = self.illustrate_path(display_image)

            self.draw_objects(display_image)
            self.draw_detected_objects(display_image)

            display_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
            height, width, channel = display_image.shape
            step = channel * width
            q_img = QtGui.QImage(display_image.data, width, height, step, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(q_img)
            self.map_item.setPixmap(pixmap)

    def illustrate_path(self, image) -> None:
        if self.state_refs_np is None or self.attributes_np is None:
            return image

        image_copy = image.copy()
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
            radius = 2
            color = (0, 255, 255)  # Default color for normal
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

            cv2.circle(image_copy, (int(self.state_refs_np[0, i] / 20.696 * image.shape[1]),
                                    int((13.786 - self.state_refs_np[1, i]) / 13.786 * image.shape[0])),
                       radius=radius, color=color, thickness=-1)

        # Add legend to the image
        legend_x, legend_y = image.shape[1] // 2 - 150, image.shape[0] // 2 - 150  # Center of the image
        legend_height = 20  # Height of each legend row
        padding = 10

        for i, (label, color) in enumerate(ATTRIBUTES.items()):
            rect_y = legend_y + i * (legend_height + padding)
            cv2.rectangle(image_copy, (legend_x, rect_y),
                          (legend_x + 20, rect_y + legend_height), color, -1)
            cv2.putText(image_copy, label, (legend_x + 30, rect_y + legend_height - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

        return image_copy

    def get_key_from_value(self, value):
        return self.reverse_object_dict.get(value, None)

    def draw_objects(self, image):
        if self.show_gt:
            for index, row in self.data.iterrows():
                x, y, entity_type, orientation = row['X'], row['Y'], row['Type'], row['Orientation']

                pixel_x = int(x * (image.shape[1] / 20.696))
                pixel_y = int((13.786 - y) * (image.shape[0] / 13.786))
                # orientation = 2 * np.pi - orientation
                orientation = - orientation

                if entity_type == 'Intersection':
                    if self.show_signs:
                        self.draw_intersection(image, pixel_x, pixel_y, orientation, 20)
                elif entity_type == 'Lane':
                    if self.show_lanes:
                        self.draw_lane(image, pixel_x, pixel_y, orientation)
                elif entity_type == 'Car':
                    if self.show_cars:
                        self.draw_car(image, pixel_x, pixel_y, orientation, steer=0.2)
                elif entity_type == 'Destination':
                    if self.show_destinations:
                        size = int(0.15 / 20.696 * 800 * self.scale_factor)
                        cv2.circle(image, (pixel_x, pixel_y), size, (235, 206, 135), -1)
                elif entity_type == 'Light':
                    if self.show_signs:
                        sign_index = self.get_key_from_value(entity_type)
                        self.draw_sign(image, pixel_x, pixel_y, orientation, self.sign_size, sign_index)
                else:
                    if self.show_signs:
                        sign_index = self.get_key_from_value(entity_type)
                        self.draw_sign(image, pixel_x, pixel_y, orientation, self.sign_size, sign_index)

    def draw_detected_objects(self, image):
        if True:
            if self.waypoints is not None:
                for i in range(0, len(self.waypoints) - 1, 8):
                    center = (int(self.waypoints[i] / 20.696 * image.shape[1]), int((13.786 - self.waypoints[i + 1]) / 13.786 * image.shape[0]))
                    cv2.circle(image, center, radius=1, color=(0, 255, 255), thickness=-1)
            if self.detected_data is None or len(self.detected_data) == 0:
                return
            x = self.detected_data[0, self.road_msg_dict['x']]
            y = self.detected_data[0, self.road_msg_dict['y']]
            yaw = self.detected_data[0, self.road_msg_dict['orientation']]
            z = self.detected_data[0, self.road_msg_dict['z']]
            self.position_label.setText(f'Position: (x: {x:.2f}, y: {y:.2f}, yaw: {(yaw / np.pi * 180):.2f}, z: {z:.2f})')
            speed = self.detected_data[0, self.road_msg_dict['speed']]
            self.speed_label.setText(f'Speed: {speed:.2f} m/s')
            self.main_window.car_widget.set_car_data(yaw / np.pi * 180, x, y, z)
            self.main_window.meter_widget.set_yaw(yaw / np.pi * 180)
            self.main_window.meter_widget.set_speed(speed * 100)
            for i in range(len(self.detected_data)):
                obj_type = self.detected_data[i, self.road_msg_dict['type']]
                x = self.detected_data[i, self.road_msg_dict['x']]
                y = self.detected_data[i, self.road_msg_dict['y']]
                orientation = self.detected_data[i, self.road_msg_dict['orientation']]

                # Convert map coordinates to pixel coordinates
                pixel_x = int(x * (image.shape[1] / 20.696))
                pixel_y = int((13.786 - y) * (image.shape[0] / 13.786))
                # orientation = 2 * np.pi - orientation
                orientation = - orientation

                if self.object_dict[obj_type] == 'Car':
                    if i == 0:
                        self.draw_car(image, pixel_x, pixel_y, orientation, steer=0.2, car_color=(0, 0, 255))
                    else:
                        self.draw_car(image, pixel_x, pixel_y, orientation, steer=0.2)
                else:
                    self.draw_sign(image, pixel_x, pixel_y, orientation, self.sign_size, obj_type)

    def draw_car(self, image, x, y, yaw, steer=23.0, car_color=(0, 255, 255), wheel_color=(0, 0, 0)):
        LENGTH = 4.5 * 0.108 * 800 * self.scale_factor / 20.696  # Car length
        WIDTH = 2.0 * 0.108 * 800 * self.scale_factor / 20.696  # Car width
        # BACKTOWHEEL = 1.0 * 0.108 * 800 * self.scale_factor / 20.696  # Distance from back to the wheel
        WHEEL_LEN = 0.6 * 0.108 * 800 * self.scale_factor / 20.696  # Length of the wheel
        WHEEL_WIDTH = 0.2 * 0.108 * 800 * self.scale_factor / 20.696  # Width of the wheel
        TREAD = 0.7 * 0.108 * 800 * self.scale_factor / 20.696  # Distance between left and right wheels
        WB = 0.258 * 800 * self.scale_factor / 20.696  # Wheelbase: distance between the front and rear wheels
        half_length = LENGTH / 2
        half_width = WIDTH / 2
        # yaw = np.pi * 0.25
        # Define the outline of the car
        # outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
        #                     [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])
        outline = np.array([[-half_length, half_length, half_length, -half_length, -half_length],
                            [half_width, half_width, -half_width, -half_width, half_width]])

        fr_wheel = np.array([[WHEEL_LEN, -WHEEL_LEN, -WHEEL_LEN, WHEEL_LEN, WHEEL_LEN],
                            [-WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, WHEEL_WIDTH - TREAD, -WHEEL_WIDTH - TREAD]])
        rr_wheel = np.copy(fr_wheel)
        fl_wheel = np.copy(fr_wheel)
        rl_wheel = np.copy(fr_wheel)

        fl_wheel[1, :] *= -1  # Flip the y-coordinates for the left wheels
        rl_wheel[1, :] *= -1

        Rot1 = np.array([[math.cos(yaw), math.sin(yaw)], [-math.sin(yaw), math.cos(yaw)]])
        Rot2 = np.array([[math.cos(steer), math.sin(steer)], [-math.sin(steer), math.cos(steer)]])

        fr_wheel = (fr_wheel.T.dot(Rot2)).T
        fl_wheel = (fl_wheel.T.dot(Rot2)).T
        fr_wheel[0, :] += WB  # Translate front wheels forward
        fl_wheel[0, :] += WB

        fr_wheel = (fr_wheel.T.dot(Rot1)).T
        fl_wheel = (fl_wheel.T.dot(Rot1)).T
        rr_wheel = (rr_wheel.T.dot(Rot1)).T
        rl_wheel = (rl_wheel.T.dot(Rot1)).T
        outline = (outline.T.dot(Rot1)).T

        outline[0, :] += x
        outline[1, :] += y
        fr_wheel[0, :] += x
        fr_wheel[1, :] += y
        rr_wheel[0, :] += x
        rr_wheel[1, :] += y
        fl_wheel[0, :] += x
        fl_wheel[1, :] += y
        rl_wheel[0, :] += x
        rl_wheel[1, :] += y

        def to_int_coords(shape):
            return np.array(shape.T, dtype=np.int32).reshape((-1, 1, 2))

        cv2.polylines(image, [to_int_coords(outline)], isClosed=True, color=car_color, thickness=2)

        cv2.polylines(image, [to_int_coords(fr_wheel)], isClosed=True, color=wheel_color, thickness=2)
        cv2.polylines(image, [to_int_coords(rr_wheel)], isClosed=True, color=wheel_color, thickness=2)
        cv2.polylines(image, [to_int_coords(fl_wheel)], isClosed=True, color=wheel_color, thickness=2)
        cv2.polylines(image, [to_int_coords(rl_wheel)], isClosed=True, color=wheel_color, thickness=2)

        cv2.circle(image, (int(x), int(y)), int(WIDTH / 2.5), (143, 28, 90), -1)

        self.draw_arrow(image, x, y, -yaw, LENGTH * 1.2)

    def draw_intersection(self, image, x, y, orientation, size):
        x, y = int(x), int(y)
        length = max(1, size)
        self.draw_arrow(image, x, y, -orientation, 20)
        orientation += np.pi / 2
        while orientation > 2 * np.pi:
            orientation -= 2 * np.pi
        while orientation < 0:
            orientation += 2 * np.pi
        if orientation == 0 or orientation == np.pi or orientation == 2 * np.pi:
            cv2.line(image, (x - length // 2, y), (x + length // 2, y), (255, 0, 0), 2)
        else:
            cv2.line(image, (x, y - length // 2), (x, y + length // 2), (255, 0, 0), 2)

    def draw_lane(self, image, x, y, orientation):
        x, y = int(x), int(y)
        while orientation > 2 * np.pi:
            orientation -= 2 * np.pi
        while orientation < 0:
            orientation += 2 * np.pi
        if orientation == 0 or orientation == 2 * np.pi:
            cv2.line(image, (0, y), (image.shape[1], y), (0, 255, 0), 2)
        elif orientation == np.pi:
            cv2.line(image, (0, y), (image.shape[1], y), (0, 0, 255), 2)
        elif orientation == np.pi / 2:
            cv2.line(image, (x, 0), (x, image.shape[0]), (255, 0, 0), 2)
        elif orientation == 3 * np.pi / 2:
            cv2.line(image, (x, 0), (x, image.shape[0]), (0, 255, 255), 2)

    def draw_arrow(self, image, x, y, orientation, size):
        x, y = int(x), int(y)
        arrow_length = max(1, size)  # Dynamic size from trackbar
        x_end = int(x + arrow_length * np.cos(orientation))
        y_end = int(y - arrow_length * np.sin(orientation))
        cv2.arrowedLine(image, (x, y), (x_end, y_end), (0, 0, 255), 2, tipLength=0.3)

    def draw_sign(self, image, x, y, orientation, size, sign_type):
        self.draw_arrow(image, x, y, -orientation, 20)
        img = self.sign_images[int(sign_type)]
        if img is not None:
            size = max(5, size)
            img = cv2.resize(img, (size, size))  # Resize sign based on trackbar value

            # Rotate the image according to orientation (optional)
            # center = (img.shape[1] // 2, img.shape[0] // 2)
            # rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(orientation), 1.0)
            # rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(0), 1.0)
            # img = cv2.warpAffine(img, rotation_matrix, (img.shape[1], img.shape[0]))

            x_start = max(0, x - size // 2)
            y_start = max(0, y - size // 2)
            x_end = min(image.shape[1], x_start + img.shape[1])
            y_end = min(image.shape[0], y_start + img.shape[0])

            sign_x_end = x_end - x_start
            sign_y_end = y_end - y_start

            image[y_start:y_end, x_start:x_end] = img[:sign_y_end, :sign_x_end]

    def eventFilter(self, source, event) -> None:
        if event.type() == QtCore.QEvent.Wheel:
            zoom_in_factor = 1.25
            zoom_out_factor = 1 / zoom_in_factor

            if event.angleDelta().y() > 0:
                factor = zoom_in_factor
            else:
                factor = zoom_out_factor

            new_zoom = self.current_zoom * factor
            if new_zoom < self.min_zoom:
                factor = self.min_zoom / self.current_zoom
                new_zoom = self.min_zoom

            self.graphics_view.scale(factor, factor)
            self.current_zoom = new_zoom
            return True
        return super().eventFilter(source, event)

    def mousePressEvent(self, event) -> None:
        if event.button() == QtCore.Qt.RightButton:
            self.clear_markers()
        elif event.button() == QtCore.Qt.LeftButton:
            self.handle_left_click(event)

    def clear_markers(self):
        for marker in self.markers:
            if marker.scene() == self.scene:
                self.scene.removeItem(marker)
        self.markers.clear()
        self.cursor_coords.clear()

    def handle_left_click(self, event) -> None:
        scene_pos = self.graphics_view.mapToScene(event.pos())
        image_x = scene_pos.x()
        image_y = scene_pos.y()

        if 0 <= image_x <= self.image_width and 0 <= image_y <= self.image_height:
            click_x = image_x * (self.image_width_real / self.image_width)
            click_y = 13.786 - image_y * (self.image_height_real / self.image_height)

            self.cursor_coords.append((click_x, click_y))
            self.cursor_x = click_x
            self.cursor_y = click_y
            self.add_marker(image_x, image_y)

    def add_marker(self, x, y) -> None:
        # Create new markers
        cursor_radius = 10
        pen = QtGui.QPen(QtGui.QColor(0, 150, 255), 2)

        circle = self.scene.addEllipse(
            x - cursor_radius,
            y - cursor_radius,
            cursor_radius * 2,
            cursor_radius * 2,
            pen
        )
        self.markers.append(circle)

        marker_size = 20
        red_pen = QtGui.QPen(QtGui.QColor(255, 0, 0), 3)

        x_line1 = self.scene.addLine(
            x - marker_size / 2,
            y - marker_size / 2,
            x + marker_size / 2,
            y + marker_size / 2,
            red_pen
        )
        x_line2 = self.scene.addLine(
            x - marker_size / 2,
            y + marker_size / 2,
            x + marker_size / 2,
            y - marker_size / 2,
            red_pen
        )
        self.markers.extend([x_line1, x_line2])

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
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to send waypoints service call")
        except Exception as e:
            raise e


class CustomGraphicsView(QGraphicsView):
    def __init__(self, parent=None):
        super().__init__(parent)
        self.setDragMode(QGraphicsView.ScrollHandDrag)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)  # Key setting
        self.setVerticalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setHorizontalScrollBarPolicy(QtCore.Qt.ScrollBarAlwaysOff)
        self.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.setMaximumHeight(800)
        self.setRenderHints(
            QtGui.QPainter.Antialiasing | QtGui.QPainter.SmoothPixmapTransform
        )

        # Zoom control variables
        self.zoom_factor = 1.25
        self.min_zoom = 0.1
        self.max_zoom = 10
        self.current_zoom = 1.0

    def wheelEvent(self, event):
        # Determine zoom direction
        if event.angleDelta().y() > 0:
            new_zoom = self.current_zoom * self.zoom_factor
        else:
            new_zoom = self.current_zoom / self.zoom_factor

        # Apply zoom constraints
        if new_zoom < self.min_zoom or new_zoom > self.max_zoom:
            return

        # Perform scaling
        self.current_zoom = new_zoom
        self.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.setResizeAnchor(QGraphicsView.AnchorUnderMouse)
        self.scale(self.zoom_factor if event.angleDelta().y() > 0 else 1 / self.zoom_factor,
                   self.zoom_factor if event.angleDelta().y() > 0 else 1 / self.zoom_factor)

        event.accept()
