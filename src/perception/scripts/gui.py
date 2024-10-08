#!/usr/bin/env python3
import sys
import cv2
import pandas as pd
import numpy as np
import os
import math
import rospy
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QHBoxLayout, QSlider, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem
from PyQt5.QtCore import Qt, QTimer
from PyQt5.QtGui import QImage, QPixmap
from std_msgs.msg import Float32MultiArray
from utils.srv import waypoints, waypointsRequest, waypointsResponse

class OpenCVGuiApp(QWidget):
    def __init__(self):
        super().__init__()
        self.current_zoom = 1.0
        self.min_zoom = 1.0
        
        # Dimensions
        self.scale_factor = 1.5
        self.image_width_real = 20.696
        self.image_height_real = 13.786
        self.image_width = int(800 * self.scale_factor)
        self.image_height = int(533 * self.scale_factor)
        
        # Set up the window
        self.setWindowTitle('BFMC Dashboard')
        self.setGeometry(100, 100, int(self.image_width*1.1), int(self.image_height*1.2))

        # Set up layout
        self.layout = QVBoxLayout()

        # Map Display
        # self.map_label = QLabel(self)
        # self.layout.addWidget(self.map_label)

        # Graphics View for zoom and pan
        self.graphics_view = QGraphicsView(self)
        self.graphics_view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.graphics_view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.layout.addWidget(self.graphics_view)

        # Scene to display the map
        self.scene = QGraphicsScene(self)
        self.graphics_view.setScene(self.scene)
        self.map_item = QGraphicsPixmapItem()
        self.scene.addItem(self.map_item)
        
        # Set up buttons
        self.button_layout = QHBoxLayout()
        self.toggle_visibility_button = QPushButton('Toggle Visibility')
        self.toggle_lanes_button = QPushButton('Toggle Lanes')
        self.toggle_cars_button = QPushButton('Toggle Cars')
        self.toggle_destinations_button = QPushButton('Toggle Destinations')
        self.toggle_path_button = QPushButton('Toggle Path')
        self.toggle_gt_button = QPushButton('Toggle Ground Truth')

        self.button_layout.addWidget(self.toggle_visibility_button)
        self.button_layout.addWidget(self.toggle_lanes_button)
        self.button_layout.addWidget(self.toggle_cars_button)
        self.button_layout.addWidget(self.toggle_destinations_button)
        self.button_layout.addWidget(self.toggle_path_button)
        self.button_layout.addWidget(self.toggle_gt_button)

        self.layout.addLayout(self.button_layout)

        # Set the main layout
        self.setLayout(self.layout)

        # Connect buttons to functions
        self.toggle_visibility_button.clicked.connect(self.toggle_visibility)
        self.toggle_lanes_button.clicked.connect(self.toggle_lanes)
        self.toggle_cars_button.clicked.connect(self.toggle_cars)
        self.toggle_destinations_button.clicked.connect(self.toggle_destinations)
        self.toggle_path_button.clicked.connect(self.toggle_path)
        self.toggle_gt_button.clicked.connect(self.toggle_gt)
        
        # Add slider for sign size adjustment
        self.sign_size_slider = QSlider(Qt.Horizontal)
        self.sign_size_slider.setRange(5, 100)
        self.sign_size_slider.setValue(20)
        self.sign_size_slider.valueChanged.connect(self.update_sign_size)
        self.layout.addWidget(self.sign_size_slider)

        # ROS-related attributes
        self.show_elements = True
        self.show_lanes = True
        self.show_cars = True
        self.show_destinations = True
        self.show_path = True
        self.show_gt = True
        self.state_refs_np = None
        self.attributes_np = None
        self.sign_size = 20

        # Load map image
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        assets_dir = os.path.join(self.current_dir, 'assets')
        self.map_image = cv2.imread(os.path.join(assets_dir, 'map1.png'))
        self.map_image = cv2.resize(self.map_image, (self.image_width, self.image_height))
        
        self.data = pd.read_csv(os.path.join(assets_dir, 'coordinates_with_context.csv'))

        rospy.init_node('visualizer', anonymous=True)
        self.call_waypoint_service('25', 'speedrun', 11.8, 2.02, 0.0)

        # Timer to update the map display
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100)
        
        # Mouse events for zoom
        self.graphics_view.viewport().installEventFilter(self)
        
        # Objects
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
            9: "Traffic Light",
            10: "Block",
            11: "Pedestrian",
            12: "Car"
        }
        self.road_object_sub = rospy.Subscriber('/road_objects', Float32MultiArray, self.road_objects_callback)
        self.road_msg_length = 6
        self.road_msg_dict = {
            'type': 0,
            'x': 1,
            'y': 2,
            'orientation': 3,
            'speed': 4,
            'confidence': 5
        }
        self.detected_data = None

    # ROS callback functions
    def road_objects_callback(self, msg):
        self.detected_data = np.array(msg.data).reshape(-1, self.road_msg_length)
        
    def toggle_visibility(self):
        self.show_elements = not self.show_elements

    def toggle_lanes(self):
        self.show_lanes = not self.show_lanes

    def toggle_cars(self):
        self.show_cars = not self.show_cars

    def toggle_destinations(self):
        self.show_destinations = not self.show_destinations

    def toggle_path(self):
        self.show_path = not self.show_path
    
    def toggle_gt(self):
        self.show_gt = not self.show_gt
        
    def update_sign_size(self, value):
        self.sign_size = value

    def call_waypoint_service(self, vref_name, path_name, x0, y0, yaw0):
        rospy.wait_for_service('waypoint_path')
        try:
            waypoint_path_service = rospy.ServiceProxy('waypoint_path', waypoints)
            req = waypointsRequest()
            req.vrefName = vref_name
            req.pathName = path_name
            req.x0 = x0
            req.y0 = y0
            req.yaw0 = yaw0

            res = waypoint_path_service(req)

            self.state_refs_np = np.array(res.state_refs.data).reshape(-1, 3).T
            self.attributes_np = np.array(res.wp_attributes.data)
            print("Service call successful.")
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")

    def illustrate_path(self, image):
        if self.state_refs_np is None or self.attributes_np is None:
            return image
        image_copy = image.copy()
        for i in range(0, self.state_refs_np.shape[1], 8):
            radius = 2
            color = (0, 255, 255)
            if self.attributes_np[i] == 4 or self.attributes_np[i] == 5:  # hard waypoints
                color = (0, 0, 255)
            elif self.attributes_np[i] == 1:  # crosswalk
                color = (0, 255, 0)
            elif self.attributes_np[i] == 9:
                color = (255, 0, 0)
            elif self.attributes_np[i] == 7:
                color = (255, 255, 0)
            elif self.attributes_np[i] == 6:
                color = (255, 255, 255)
            elif self.attributes_np[i] >= 100:
                color = (0, 165, 255)
            elif self.attributes_np[i] == 2 or self.attributes_np[i] == 102:
                color = (255, 0, 255)
            cv2.circle(image_copy, (int(self.state_refs_np[0, i] / 20.696 * image.shape[1]),
                                    int((13.786 - self.state_refs_np[1, i]) / 13.786 * image.shape[0])),
                       radius=int(radius), color=color, thickness=-1)
        return image_copy

    def draw_objects(self, image):
        if self.show_gt:
            for index, row in self.data.iterrows():
                x, y, entity_type, orientation = row['X'], row['Y'], row['Type'], row['Orientation']

                # Convert map coordinates to pixel coordinates
                pixel_x = int(x * (image.shape[1] / 20.696))
                pixel_y = int((13.786 - y) * (image.shape[0] / 13.786))
                # orientation = 2 * np.pi - orientation
                orientation = - orientation

                if entity_type == 'Intersection':
                    if self.show_elements:
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
                else:
                    if self.show_elements:
                        self.draw_sign(image, pixel_x, pixel_y, entity_type, orientation, self.sign_size)
                        
    def draw_detected_objects(self, image):
        if True:
            if self.detected_data is None:
                return
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
                    if self.show_cars:
                        self.draw_car(image, pixel_x, pixel_y, orientation, steer=0.2)
                else:
                    entity_type = self.object_dict[obj_type]
                    if self.show_elements:
                        self.draw_sign(image, pixel_x, pixel_y, entity_type, orientation, self.sign_size)

    def draw_car(self, image, x, y, yaw, steer=23.0, car_color=(0, 255, 255), wheel_color=(0, 0, 0)):
        LENGTH = 4.5*0.108 *800*self.scale_factor/20.696 # Car length
        WIDTH = 2.0*0.108  *800*self.scale_factor/20.696 # Car width
        BACKTOWHEEL = 1.0*0.108 *800*self.scale_factor/20.696 # Distance from back to the wheel
        WHEEL_LEN = 0.6*0.108 *800*self.scale_factor/20.696 # Length of the wheel
        WHEEL_WIDTH = 0.2*0.108 *800*self.scale_factor/20.696 # Width of the wheel
        TREAD = 0.7*0.108 *800*self.scale_factor/20.696 # Distance between left and right wheels
        WB = 0.27 *800*self.scale_factor/20.696 # Wheelbase: distance between the front and rear wheels
        # yaw = np.pi * 0.25
        # Define the outline of the car
        outline = np.array([[-BACKTOWHEEL, (LENGTH - BACKTOWHEEL), (LENGTH - BACKTOWHEEL), -BACKTOWHEEL, -BACKTOWHEEL],
                            [WIDTH / 2, WIDTH / 2, - WIDTH / 2, -WIDTH / 2, WIDTH / 2]])

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

        cv2.circle(image, (int(x), int(y)), int(WIDTH/2.5), (0, 0, 0), -1)
        
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

    def draw_sign(self, image, x, y, sign_type, orientation, size):
        self.draw_arrow(image, x, y, -orientation, 20)
        try:
            img = None
            assets_path = os.path.join(self.current_dir, 'assets')
            if sign_type == 'Roundabout':
                img = cv2.imread(os.path.join(assets_path, 'roundabout.jpg'))
            elif sign_type == 'Sign':
                img = cv2.imread(os.path.join(assets_path, 'stopsign.jpg'))
            elif sign_type == 'Parking':
                img = cv2.imread(os.path.join(assets_path, 'parking.jpg'))
            elif sign_type == 'Crosswalk':
                img = cv2.imread(os.path.join(assets_path, 'crosswalk.jpg'))
            elif sign_type == 'Highway Entrance':
                img = cv2.imread(os.path.join(assets_path, 'highway_entrance.jpg'))
            elif sign_type == 'Highway Exit':
                img = cv2.imread(os.path.join(assets_path, 'highway_exit.jpg'))
            elif sign_type == 'Oneway':
                img = cv2.imread(os.path.join(assets_path, 'oneway.jpg'))
            elif sign_type == 'No Entry':
                img = cv2.imread(os.path.join(assets_path, 'noentry.jpg'))
            elif sign_type == 'Pedestrian':
                img = cv2.imread(os.path.join(assets_path, 'pedestrian.jpg'))
            elif sign_type == 'Traffic Light':
                img = cv2.imread(os.path.join(assets_path, 'trafficlight.png'))
            
            if img is not None:
                size = max(5, size)
                img = cv2.resize(img, (size, size))  # Resize sign based on trackbar value

                # Rotate the image according to orientation (optional)
                # center = (img.shape[1] // 2, img.shape[0] // 2)
                # rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(orientation), 1.0)
                # rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(0), 1.0)
                # img = cv2.warpAffine(img, rotation_matrix, (img.shape[1], img.shape[0]))

                # Calculate the boundaries for placing the image
                x_start = max(0, x - size // 2)
                y_start = max(0, y - size // 2)
                x_end = min(image.shape[1], x_start + img.shape[1])
                y_end = min(image.shape[0], y_start + img.shape[0])

                # Only overlay the part of the sign that fits in the map
                sign_x_end = x_end - x_start
                sign_y_end = y_end - y_start

                image[y_start:y_end, x_start:x_end] = img[:sign_y_end, :sign_x_end]
        except FileNotFoundError:
            print(f"Image for {sign_type} not found. Skipping.")
            
    def update_map(self):
        # Create a copy of the map to draw on
        display_image = self.map_image.copy()

        # Illustrate path if needed
        if self.show_path:
            display_image = self.illustrate_path(display_image)

        self.draw_objects(display_image)
        self.draw_detected_objects(display_image)
        
        # Convert the frame to RGB format
        display_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
        height, width, channel = display_image.shape
        step = channel * width
        q_img = QImage(display_image.data, width, height, step, QImage.Format_RGB888)
        # self.map_label.setPixmap(QPixmap.fromImage(q_img))
        pixmap = QPixmap.fromImage(q_img)
        self.map_item.setPixmap(pixmap)

    def eventFilter(self, source, event):
        if event.type() == event.Wheel:  # Zoom with mouse wheel
            zoom_in_factor = 1.25
            zoom_out_factor = 1 / zoom_in_factor

            if event.angleDelta().y() > 0:
                factor = zoom_in_factor
            else:
                factor = zoom_out_factor

            # Calculate the new zoom level
            new_zoom = self.current_zoom * factor

            # Only apply zoom if it does not go below the starting level
            if new_zoom < self.min_zoom:
                factor = self.min_zoom / self.current_zoom
                new_zoom = self.min_zoom

            self.graphics_view.scale(factor, factor)
            self.current_zoom = new_zoom

        return super().eventFilter(source, event)
            
    def closeEvent(self, event):
        event.accept()

if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = OpenCVGuiApp()
    window.show()
    sys.exit(app.exec_())