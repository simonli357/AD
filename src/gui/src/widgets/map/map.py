from PyQt5 import QtWidgets, QtCore, QtGui
from PyQt5.QtWidgets import QLabel
from std_srvs.srv import TriggerResponse
from .view import GraphicsView
from ..enums import MapData

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
        self.max_zoom = 8.0
        self.markers = []
        self.cursor_coords = []
        self.cursor_x = 3.86
        self.cursor_y = 3.62

        self.show_signs = False
        self.show_lanes = False
        self.show_cars = False
        self.show_destinations = True
        self.show_path = True
        self.show_nodes = True
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

        self.setup_ui()

        # Map configuration
        self.image_width_real = MapData.REAL_WORLD_WIDTH.value
        self.image_height_real = MapData.REAL_WORLD_HEIGHT.value
        self.image_width = MapData.PNG_WIDTH.value
        self.image_height = MapData.PNG_HEIGHT.value
        self.real_x_per_pixel = MapData.REAL_X_PER_PIXEL.value
        self.real_y_per_pixel = MapData.REAL_Y_PER_PIXEL.value

        # Initialize map image
        self.load_map_image()

        # Timer for updates
        self.realtime_timer = QtCore.QTimer(self)
        self.realtime_timer.timeout.connect(self.update_detected_objects)
        self.realtime_timer.start(100)

        self.mouse_updates = 0
        self.timer = QtCore.QTimer(self)
        self.timer.setInterval(100)
        self.timer.timeout.connect(self.update_scene)
        self.timer.start()

    def setup_ui(self) -> None:
        self.layout = QtWidgets.QVBoxLayout(self)
        self.layout.setAlignment(QtCore.Qt.AlignLeft)
        self.layout.setContentsMargins(0, 0, 0, 0)

        # Graphics View setup
        self.graphics_view = GraphicsView(self)
        self.layout.addWidget(self.graphics_view)
        self.graphics_view.viewport().installEventFilter(self)
        self.graphics_view.setMouseTracking(True)
        self.graphics_view.viewport().setMouseTracking(True)

        # Cursor Pos Label
        self.cursor_coords_label = QLabel(self.graphics_view)
        self.cursor_coords_label.setStyleSheet("""
            border: none;
            background-color: rgba(0, 0, 0, 0.7);
            color: #00ff00;
            font-size: 16px;
        """)
        self.cursor_coords_label.hide()
        self.cursor_coords_label.setAlignment(QtCore.Qt.AlignCenter)

        # Scene setup
        self.scene = QtWidgets.QGraphicsScene(self)
        self.graphics_view.setScene(self.scene)
        self.map_item = QtWidgets.QGraphicsPixmapItem()
        self.scene.addItem(self.map_item)

    def load_map_image(self) -> None:
        self.image_width = max(100, self.width())
        self.image_height = max(100, self.height())
        self.real_x_per_pixel = self.image_width_real / self.image_width
        self.real_y_per_pixel = self.image_height_real / self.image_height
        map_image = cv2.imread(os.path.join(self.assets_dir, 'map1.png'))
        self.map_image = cv2.resize(map_image, (self.image_width, self.image_height))
        self.empty_map_image = self.map_image.copy()
        self.scene.setSceneRect(0, 0, self.image_width, self.image_height)
        self.update_map_display()

    def resizeEvent(self, event):
        self.load_map_image()
        super().resizeEvent(event)

    def update_map_display(self) -> None:
        if hasattr(self, 'map_image') and hasattr(self, 'empty_map_image'):
            display_image = self.empty_map_image.copy()
            self.graphics_view.show_nodes()
            self.illustrate_path(display_image)
            self.draw_objects(display_image)
            self.map_image = display_image

    def update_detected_objects(self) -> None:
        if hasattr(self, 'map_image'):
            display_image = self.map_image.copy()
            self.draw_detected_objects(display_image)
            display_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
            height, width, channel = display_image.shape
            step = channel * width
            q_img = QtGui.QImage(display_image.data, width, height, step, QtGui.QImage.Format_RGB888)
            pixmap = QtGui.QPixmap.fromImage(q_img)
            self.map_item.setPixmap(pixmap)

    def illustrate_path(self, image):
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
            cv2.circle(image,
                       (int(self.state_refs_np[0, i] / self.image_width_real * self.image_width),
                        int((self.image_height_real - self.state_refs_np[1, i]) / self.image_height_real * self.image_height)),
                       radius=radius, color=color, thickness=-1)

        # Add legend to the image
        legend_x, legend_y = image.shape[1] // 2 - 150, image.shape[0] // 2 - 150  # Center of the image
        legend_height = 20  # Height of each legend row
        padding = 10

        for i, (label, color) in enumerate(ATTRIBUTES.items()):
            rect_y = legend_y + i * (legend_height + padding)
            cv2.rectangle(image, (legend_x, rect_y),
                          (legend_x + 20, rect_y + legend_height), color, -1)
            cv2.putText(image, label, (legend_x + 30, rect_y + legend_height - 5),
                        cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 255, 255), 1, cv2.LINE_AA)

    def get_key_from_value(self, value):
        return self.reverse_object_dict.get(value, None)

    def draw_objects(self, image):
        if self.show_gt:
            for index, row in self.data.iterrows():
                entity_type, orientation = row['Type'], row['Orientation']

                pixel_x = int(row['X'] / self.image_width_real * self.image_width)
                pixel_y = int((self.image_height_real - row['Y']) / self.image_height_real * self.image_height)

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
                        self.draw_car_obstacle(image, pixel_x, pixel_y, orientation, steer=0.2)
                        self.cars_drawn = True
                elif entity_type == 'Destination':
                    if self.show_destinations:
                        axes = (int(0.15 / self.image_width_real * self.image_width),
                                int(0.15 / self.image_height_real * self.image_height))
                        cv2.ellipse(image, (pixel_x, pixel_y), axes,
                                    0, 0, 360, (235, 206, 135), -1)
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

    def draw_car_obstacle(self, image, x, y, yaw, steer=23.0, car_color=None, wheel_color=None):
        # Load car image with transparency
        car_img = cv2.imread(self.car_icon_path, cv2.IMREAD_UNCHANGED)

        if car_img is None:
            return

        car_length_m = 0.64

        pixels_per_meter_x = self.image_width / self.image_width_real

        orig_height, orig_width = car_img.shape[:2]
        aspect_ratio = orig_width / orig_height

        target_width = int(car_length_m * pixels_per_meter_x)
        target_height = int(target_width / aspect_ratio)

        resized_car = cv2.resize(car_img, (target_width, target_height))

        rotation_matrix = cv2.getRotationMatrix2D(
            (target_width / 2, target_height / 2),  # Center
            math.degrees(-yaw) - 90,
            1.0  # Scale
        )

        rotated = cv2.warpAffine(resized_car, rotation_matrix, (target_width, target_height))

        if rotated.shape[2] == 4:
            mask = rotated[:, :, 3]
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask = mask.astype(float) / 255.0
            rotated = rotated[:, :, :3]
            rotated = cv2.cvtColor(rotated, cv2.COLOR_BGR2RGB)
        else:
            mask = np.ones_like(rotated, dtype=float)
            rotated = cv2.cvtColor(rotated, cv2.COLOR_BGR2RGB)

        x_start = int(x - target_width / 2)
        y_start = int(y - target_height / 2)
        x_end = x_start + target_width
        y_end = y_start + target_height

        x1 = max(0, x_start)
        y1 = max(0, y_start)
        x2 = min(image.shape[1], x_end)
        y2 = min(image.shape[0], y_end)

        if x1 >= x2 or y1 >= y2:
            return

        car_region = rotated[y1 - y_start:y2 - y_start, x1 - x_start:x2 - x_start]
        mask_region = mask[y1 - y_start:y2 - y_start, x1 - x_start:x2 - x_start]
        img_region = image[y1:y2, x1:x2]

        image[y1:y2, x1:x2] = (car_region * mask_region + img_region * (1 - mask_region)).astype(np.uint8)

        LENGTH = 0.4 * pixels_per_meter_x
        self.draw_arrow(image, x, y, -yaw, LENGTH * 1.2)

    def draw_car(self, image, x, y, yaw, steer=23.0, car_color=None, wheel_color=None):
        # Load car image with transparency
        car_img = cv2.imread(self.car_icon_path, cv2.IMREAD_UNCHANGED)

        if car_img is None:
            return

        car_length_m = 0.64

        pixels_per_meter_x = self.image_width / self.image_width_real

        orig_height, orig_width = car_img.shape[:2]
        aspect_ratio = orig_width / orig_height

        target_width = int(car_length_m * pixels_per_meter_x)
        target_height = int(target_width / aspect_ratio)

        resized_car = cv2.resize(car_img, (target_width, target_height))

        rotation_matrix = cv2.getRotationMatrix2D(
            (target_width / 2, target_height / 2),  # Center
            math.degrees(-yaw) - 90,
            1.0  # Scale
        )

        rotated = cv2.warpAffine(resized_car, rotation_matrix, (target_width, target_height))

        if rotated.shape[2] == 4:
            mask = rotated[:, :, 3]
            mask = cv2.cvtColor(mask, cv2.COLOR_GRAY2BGR)
            mask = mask.astype(float) / 255.0
            rotated = rotated[:, :, :3]
        else:
            mask = np.ones_like(rotated, dtype=float)

        x_start = int(x - target_width / 2)
        y_start = int(y - target_height / 2)
        x_end = x_start + target_width
        y_end = y_start + target_height

        x1 = max(0, x_start)
        y1 = max(0, y_start)
        x2 = min(image.shape[1], x_end)
        y2 = min(image.shape[0], y_end)

        if x1 >= x2 or y1 >= y2:
            return

        car_region = rotated[y1 - y_start:y2 - y_start, x1 - x_start:x2 - x_start]
        mask_region = mask[y1 - y_start:y2 - y_start, x1 - x_start:x2 - x_start]
        img_region = image[y1:y2, x1:x2]

        image[y1:y2, x1:x2] = (car_region * mask_region + img_region * (1 - mask_region)).astype(np.uint8)

        LENGTH = 0.4 * pixels_per_meter_x
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
        cv2.arrowedLine(image, (x, y), (x_end, y_end), (0, 255, 0), 1, tipLength=0.3)

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
            zoom_in_factor = 1.15
            zoom_out_factor = 1 / zoom_in_factor

            if event.angleDelta().y() > 0:
                factor = zoom_in_factor
            else:
                factor = zoom_out_factor

            new_zoom = self.current_zoom * factor
            if new_zoom < self.min_zoom:
                factor = self.min_zoom / self.current_zoom
                new_zoom = self.min_zoom
            if new_zoom > self.max_zoom:
                return True

            self.graphics_view.scale(factor, factor)
            self.current_zoom = new_zoom
            self.update_map_display()
            return True
        elif event.type() == QtCore.QEvent.MouseMove:
            self.mouse_pos = event.pos()
            self.mouse_updates += 1
            return False
        elif event.type() == QtCore.QEvent.Leave:
            self.cursor_coords_label.hide()
            return False
        return super().eventFilter(source, event)

    def update_scene(self):
        if hasattr(self, 'mouse_pos') and hasattr(self, 'mouse_updates'):
            if self.mouse_updates == 0:
                return
            scene_pos = self.graphics_view.mapToScene(self.mouse_pos)
            x_scene = scene_pos.x()
            y_scene = scene_pos.y()
            if 0 <= x_scene < self.image_width and 0 <= y_scene < self.image_height:
                real_x = scene_pos.x() * self.real_x_per_pixel
                real_y = scene_pos.y() * self.real_y_per_pixel
                self.cursor_coords_label.setText(f"  ({real_x:.2f}, {real_y:.2f}) ")
                self.cursor_coords_label.move(int(self.mouse_pos.x() - self.cursor_coords_label.width() / 2), int(self.mouse_pos.y() - 60))
                self.cursor_coords_label.show()
            self.update_map_display()
            self.mouse_updates -= 1

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
                    return
                retries += 1
                time.sleep(0.1)
            print("Failed to send waypoints service call")
        except Exception as e:
            raise e
