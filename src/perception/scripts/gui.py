#!/usr/bin/env python3
import sys
import cv2
import pandas as pd
import numpy as np
import os
import math
import rospy
from std_srvs.srv import Trigger, TriggerResponse
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout, QPushButton, QLabel, QHBoxLayout, QSlider, QGraphicsView, QGraphicsScene, QGraphicsPixmapItem, QSpacerItem, QSizePolicy, QTextEdit
from PyQt5.QtCore import Qt, QTimer, QPointF
from PyQt5.QtGui import QImage, QPixmap, QPen, QColor, QCursor
from std_msgs.msg import Float32MultiArray, String
from std_srvs.srv import SetBool, SetBoolRequest
from utils.srv import waypoints, waypointsRequest, goto_command, goto_commandRequest, set_states, set_statesRequest
from utils.msg import Lane2
from cv_bridge import CvBridge
from sensor_msgs.msg import Image

class OpenCVGuiApp(QWidget):
    def __init__(self):
        super().__init__()
        self.current_zoom = 1.0
        self.min_zoom = 1.0
        
        self.scale_factor = 1.5
        self.image_width_real = 20.696
        self.image_height_real = 13.786
        self.image_width = int(800 * self.scale_factor)
        self.image_height = int(533 * self.scale_factor)
        
        self.setWindowTitle('BFMC Dashboard')
        self.setGeometry(100, 100, 1500, 800)

        self.layout = QHBoxLayout()
        
        # Left Panel - Map Display
        self.left_panel_layout = QVBoxLayout()
        self.left_panel_widget = QWidget()
        # self.left_panel_widget.setFixedSize(1000, 800)
        self.left_panel_widget.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.left_panel_widget.setLayout(self.left_panel_layout)

        # Graphics View for zoom and pan
        self.graphics_view = QGraphicsView(self)
        self.graphics_view.setDragMode(QGraphicsView.ScrollHandDrag)
        self.graphics_view.setTransformationAnchor(QGraphicsView.AnchorUnderMouse)
        self.graphics_view.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.left_panel_layout.addWidget(self.graphics_view)

        # Scene to display the map
        self.scene = QGraphicsScene(self)
        self.graphics_view.setScene(self.scene)
        self.map_item = QGraphicsPixmapItem()
        self.scene.addItem(self.map_item)
        
        # Set up buttons
        self.toggle_button_layout = QHBoxLayout()
        self.toggle_signs_button = QPushButton('Toggle Signs')
        self.toggle_lanes_button = QPushButton('Toggle Lanes')
        self.toggle_cars_button = QPushButton('Toggle Cars')
        self.toggle_destinations_button = QPushButton('Toggle Destinations')
        self.toggle_path_button = QPushButton('Toggle Path')
        self.toggle_gt_button = QPushButton('Toggle GT')
        self.toggle_depth_button = QPushButton('Toggle Depth')
        self.toggle_button_layout.addWidget(self.toggle_signs_button)
        self.toggle_button_layout.addWidget(self.toggle_lanes_button)
        self.toggle_button_layout.addWidget(self.toggle_cars_button)
        self.toggle_button_layout.addWidget(self.toggle_destinations_button)
        self.toggle_button_layout.addWidget(self.toggle_path_button)
        self.toggle_button_layout.addWidget(self.toggle_gt_button)
        self.toggle_button_layout.addWidget(self.toggle_depth_button)
        self.set_states_button = QPushButton('Set States')
        self.toggle_button_layout.addWidget(self.set_states_button)
        
        self.left_panel_layout.addLayout(self.toggle_button_layout)
        
        self.control_button_layout = QHBoxLayout()
        self.start_button = QPushButton('Start')
        self.goto_button = QPushButton('Go to')
        self.started = False
        self.control_button_layout.addWidget(self.start_button)
        self.control_button_layout.addWidget(self.goto_button)

        # Connect buttons to functions
        self.toggle_signs_button.clicked.connect(self.toggle_visibility)
        self.toggle_lanes_button.clicked.connect(self.toggle_lanes)
        self.toggle_cars_button.clicked.connect(self.toggle_cars)
        self.toggle_destinations_button.clicked.connect(self.toggle_destinations)
        self.toggle_path_button.clicked.connect(self.toggle_path)
        self.toggle_gt_button.clicked.connect(self.toggle_gt)
        self.toggle_depth_button.clicked.connect(self.toggle_depth)
        self.start_button.clicked.connect(self.start)
        self.goto_button.clicked.connect(self.goto)
        self.set_states_button.clicked.connect(self.set_states)
        
        # Add slider for sign size adjustment
        self.sign_size_slider = QSlider(Qt.Horizontal)
        self.sign_size_slider.setRange(5, 100)
        self.sign_size_slider.setValue(20)
        # self.left_panel_layout.addWidget(self.sign_size_slider)
        
        # Add left panel to main layout
        self.layout.addWidget(self.left_panel_widget)
        
        # Right Panel - Camera Feed and Vehicle Info
        self.right_panel_layout = QVBoxLayout()
        self.right_panel_widget = QWidget()
        self.camera_w = int(640 /640 * 500)
        self.camera_h = int(480 / 640 * 500)
        self.right_panel_widget.setFixedSize(self.camera_w, 800)
        self.right_panel_widget.setLayout(self.right_panel_layout)
        
        # Camera feed label
        self.camera_label = QLabel(self)
        self.camera_label.setFixedSize(self.camera_w, self.camera_h)
        self.right_panel_layout.addWidget(self.camera_label)
        self.text_layout = QVBoxLayout()
        self.text_time_layout = QHBoxLayout()
        # Vehicle information labels
        self.position_label = QLabel('Position: (x: 0.0, y: 0.0, yaw: 0.0, z: 0.0)')
        self.cursor_label = QLabel('Cursor: (x: 0.0, y: 0.0, yaw: 0.0)')
        self.cursor_x = 3.86
        self.cursor_y = 3.62
        self.speed_label = QLabel('Speed: 0.0 m/s')
        self.text_layout.addWidget(self.position_label)
        self.text_layout.addWidget(self.cursor_label)
        self.text_layout.addWidget(self.speed_label)
        self.text_time_layout.addLayout(self.text_layout)
        #timer label
        self.timer_label = QLabel('00:00:00')
        self.timer_label.setAlignment(Qt.AlignCenter)
        self.timer_label.setStyleSheet(
            """
            font-size: 24px;
            font-weight: bold;
            color: #4CAF50;
            border: 2px solid #4CAF50;
            border-radius: 10px;
            padding: 5px;
            """
        )
        self.centiseconds = 0
        self.time_timer = QTimer(self)
        self.time_timer.timeout.connect(self.update_stopwatch)
        # self.time_timer.start(10)  # Update every 10 ms
        self.text_time_layout.addWidget(self.timer_label)
        
        self.right_panel_layout.addLayout(self.text_time_layout)
        self.right_panel_layout.addLayout(self.control_button_layout)
        
        # Add a text area for status messages
        self.message_display = QTextEdit()
        self.message_display.setReadOnly(True)
        self.right_panel_layout.addWidget(self.message_display)
        self.message_history = []
        
        # Add right panel to main layout
        self.layout.addWidget(self.right_panel_widget)
        self.setLayout(self.layout)
        
        # Timer to update the map display
        self.timer = QTimer(self)
        self.timer.timeout.connect(self.update_map)
        self.timer.start(100) # Update every 100 ms
        
        # Mouse events for zoom
        self.graphics_view.viewport().installEventFilter(self)
        
        dark_style = """
            QWidget {
                background-color: #1E1E1E;  /* Dark background */
                color: #FFFFFF;  /* White text */
                font-family: 'Roboto', sans-serif;  /* Modern font */
                font-size: 14px;
            }

            QPushButton {
                background-color: #0CD2EE;  /* Slightly lighter button background */
                border: 2px solid #3E3E3E;  /* Subtle border */
                border-radius: 8px;  /* Rounded buttons */
                color: #FFFFFF;  /* White text */
                padding: 10px;
            }

            QPushButton:hover {
                background-color: #3A3A3A;  /* Change button color on hover */
                border-color: #5E5E5E;  /* Lighter border on hover */
            }

            QPushButton:pressed {
                background-color: #007BFF;  /* Blue accent when pressed */
                border-color: #0056b3;
            }

            QSlider::groove:horizontal {
                border: 1px solid #444;
                height: 8px;
                background: #2A2A2A;  /* Dark slider track */
                border-radius: 4px;
            }

            QSlider::handle:horizontal {
                background: #007BFF;  /* Blue slider handle */
                border: 1px solid #0056b3;
                width: 14px;
                margin: -4px 0;  /* Center the handle */
                border-radius: 7px;  /* Rounded handle */
            }

            QLabel {
                color: #BBBBBB;  /* Slightly dim text for labels */
            }

            QGraphicsView {
                border: none;
            }

            QTextEdit {
                background-color: #2A2A2A;
                border: 2px solid #3E3E3E;
                color: #FFFFFF;
            }
        """
        self.setStyleSheet(dark_style)

        text_style = """
            QLabel {
                color: #00FF00;  /* Neon green text */
                font-family: 'Roboto', sans-serif;
                font-size: 14px;
            }
        """
        self.position_label.setStyleSheet(text_style)
        self.cursor_label.setStyleSheet(text_style)
        self.speed_label.setStyleSheet(text_style)

        panel_border_style = """
            QWidget {
                border: 2px solid #007BFF;
                border-radius: 10px;
                background-color: #1E1E1E;
            }
        """
        self.left_panel_widget.setStyleSheet(panel_border_style)
        self.right_panel_widget.setStyleSheet(panel_border_style)
        
        gradient_button_style = """
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #007BFF, stop:1 #00FFFF);  /* Blue to cyan gradient */
                border: 2px solid #007BFF;
                border-radius: 12px;
                color: white;
                padding: 10px;
                font-weight: bold;
                font-size: 14px;
            }

            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #00FFFF, stop:1 #007BFF);  /* Reverse gradient on hover */
            }

            QPushButton:pressed {
                background-color: #0056b3;
            }
        """
        self.toggle_signs_button.setStyleSheet(gradient_button_style)
        self.toggle_cars_button.setStyleSheet(gradient_button_style)
        self.toggle_lanes_button.setStyleSheet(gradient_button_style)
        self.toggle_destinations_button.setStyleSheet(gradient_button_style)
        self.toggle_path_button.setStyleSheet(gradient_button_style)
        self.toggle_gt_button.setStyleSheet(gradient_button_style)
        self.toggle_depth_button.setStyleSheet(gradient_button_style)
        gradient_button_style_red = """
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #DC3545, stop:1 #FF6347);  /* Red gradient for Stop */
                border: 2px solid #DC3545;  /* Red border */
                border-radius: 12px;
                color: white;
                padding: 10px;
                font-weight: bold;
                font-size: 14px;
            }

            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    /* orange on hover */
                    stop:0 #FF6347, stop:1 #DC3545);
            }

            QPushButton:pressed {
                background-color: #0056b3;
            }
        """
        self.set_states_button.setStyleSheet(gradient_button_style_red)
        circular_button_style = """
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #28A745, stop:1 #00FF00);  /* Green gradient for Start */
                border: 2px solid #28A745;  /* Green border */
                border-radius: 40px;  /* Circular shape (80px diameter) */
                color: white;
                font-size: 14px;
                width: 80px;
                height: 80px;
                padding: 10px;
            }
            
            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #00FFFF, stop:1 #007BFF);  /* Reverse gradient on hover */
            }

            QPushButton:pressed {
                background-color: #0056b3;
            }
            
            QPushButton:pressed {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #DC3545, stop:1 #FF6347);  /* Red gradient when pressed */
                border-color: #DC3545;  /* Red border */
            }
        """
        self.start_button.setStyleSheet(circular_button_style)
        gradient_button_style2 = """
            QPushButton {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #00FFFF, stop:1 #28A745);
                border: 2px solid #28A745;  /* Green border */
                border-radius: 40px;  /* Circular shape (80px diameter) */
                color: white;
                font-size: 14px;
                width: 80px;
                height: 80px;
                padding: 10px;
                font-weight: bold;
                font-size: 14px;
            }

            QPushButton:hover {
                background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                    stop:0 #00FFFF, stop:1 #007BFF);  /* Reverse gradient on hover */
            }

            QPushButton:pressed {
                background-color: #0056b3;
            }
        """
        self.goto_button.setStyleSheet(gradient_button_style2)
        
        # Button related attributes
        self.show_signs = False
        self.show_lanes = False
        self.show_cars = False
        self.show_destinations = True
        self.show_path = True
        self.show_gt = True
        self.show_depth = False
        self.state_refs_np = None
        self.attributes_np = None
        self.sign_size = 20
        self.bridge = CvBridge()

        # Load map image
        self.current_dir = os.path.dirname(os.path.abspath(__file__))
        assets_dir = os.path.join(self.current_dir, 'assets')
        self.map_image = cv2.imread(os.path.join(assets_dir, 'map1.png'))
        self.map_image = cv2.resize(self.map_image, (self.image_width, self.image_height))
        
        self.data = pd.read_csv(os.path.join(assets_dir, 'coordinates_with_context.csv'))

        rospy.init_node('visualizer', anonymous=True)
        x_init = rospy.get_param('/x_init')
        y_init = rospy.get_param('/y_init')
        yaw_init = rospy.get_param('/yaw_init')
        path_name = rospy.get_param('/pathName', default='run1')
        self.call_waypoint_service('25', path_name, x_init, y_init, yaw_init)
        
        # Objects
        # Lane
        self.center = None
        self.crosswalk = False
        self.stopline = False
        # Sign
        self.detected_data = None
        self.waypoints = None
        self.numObj = 0
        self.detected_objects = np.zeros(10)
        self.class_names = ["oneway", "highwayentrance", "stopsign", "roundabout", "park", "crosswalk", "noentry", "highwayexit", "priority", "lights", "block", "pedestrian", "car"]
        self.confidence_thresholds = [0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.7, 0.65, 0.65, 0.65, 0.65, 0.7, 0.75]
        
        self.COLOR_LIST = [
            (1, 1, 1), (0.098, 0.325, 0.850), (0.125, 0.694, 0.929), (0.556, 0.184, 0.494), (0.188, 0.674, 0.466),
            (0.933, 0.745, 0.301), (0.184, 0.078, 0.635), (0.300, 0.300, 0.300), (0.600, 0.600, 0.600), (0.000, 0.000, 1.000),
            (0.000, 0.500, 1.000), (0.000, 0.749, 0.749), (0.000, 1.000, 0.000)
        ]
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
            13: "Sign"
        }

        self.reverse_object_dict = {v: k for k, v in self.object_dict.items()}
        
        self.sign_images = []
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'oneway.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'highway_entrance.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'stopsign.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'roundabout.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'parking.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'crosswalk.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'noentry.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'highway_exit.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'priority.png')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'trafficlight.png')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'roadblock.png')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'pedestrian.png')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'car.jpg')))
        self.sign_images.append(cv2.imread(os.path.join(assets_dir, 'stopsign.jpg')))
        
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
        
        # ROS Services
        self.trigger_service = rospy.Service('/notify_params_updated', Trigger, self.update_params)
        
        # ROS Subscribers
        self.road_object_sub = rospy.Subscriber('/road_objects', Float32MultiArray, self.road_objects_callback)
        self.camera_sub = rospy.Subscriber('/camera/color/image_raw', Image, self.camera_callback)
        self.depth_sub = rospy.Subscriber('/camera/depth/image_raw', Image, self.depth_callback)
        self.waypoint_sub = rospy.Subscriber("/waypoints", Float32MultiArray, self.waypoint_callback, queue_size=3)
        self.sign_sub = rospy.Subscriber('/sign', Float32MultiArray, self.sign_callback)
        self.sign_sub = rospy.Subscriber('/lane', Lane2, self.lane_callback)
        self.message_sub = rospy.Subscriber('/message', String, self.message_callback)
        return

    # ROS service calls
    def update_params(self, req):
        try:
            # Retrieve updated parameters
            state_refs = rospy.get_param('/state_refs')
            self.state_refs_np = np.array(state_refs).reshape(3, -1)

            state_attributes = rospy.get_param('/state_attributes')
            self.attributes_np = np.array(state_attributes)

            print("state ref shape: ", self.state_refs_np.shape)
            #print first 3 rows
            print("state ref: ", self.state_refs_np.T[:, :3])
            path = os.path.dirname(os.path.abspath(__file__))
            np.savetxt(os.path.join(path,'state_refs.txt'), self.state_refs_np.T, fmt='%.4f')
            print("saved state refs")
            rospy.loginfo("Parameters updated successfully.")
            return TriggerResponse(success=True, message="Parameters updated")
        except Exception as e:
            rospy.logerr(f"Failed to update parameters: {e}")
            return TriggerResponse(success=False, message=f"Failed to update: {e}")
    def start(self):
        self.call_start_service(not self.started)
        if not self.started:
            self.time_timer.start(10)
            self.start_button.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #DC3545, stop:1 #00FF00);
                    border: 2px solid #DC3545;  
                    border-radius: 40px;  /* Circular shape (80px diameter) */
                    color: white;
                    font-size: 14px;
                    width: 80px;
                    height: 80px;
                    padding: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #00FFFF, stop:1 #28A745);
                }
                QPushButton:pressed {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #DC3545, stop:1 #FF6347);  /* Red gradient when pressed */
                    border-color: #DC3545;  /* Red border */
                }
            """)
            self.start_button.setText("Stop")
        else:
            self.time_timer.stop()
            self.start_button.setStyleSheet("""
                QPushButton {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #28A745, stop:1 #00FF00);  /* Green gradient for Start */
                    border: 2px solid #28A745;  /* Green border */
                    border-radius: 40px;  /* Circular shape (80px diameter) */
                    color: white;
                    font-size: 14px;
                    width: 80px;
                    height: 80px;
                    padding: 10px;
                }
                QPushButton:hover {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #00FFFF, stop:1 #28A745);
                }
                QPushButton:pressed {
                    background: qlineargradient(x1:0, y1:0, x2:1, y2:1,
                        stop:0 #DC3545, stop:1 #FF6347);  /* Red gradient when pressed */
                    border-color: #DC3545;  /* Red border */
                }
            """)
            self.start_button.setText("Start")
        self.started = not self.started
    def goto(self):
        self.call_goto_service(self.cursor_x, self.cursor_y)
    def set_states(self):
        self.call_set_states_service(self.cursor_x, self.cursor_y)
    def call_goto_service(self, x, y):
        print("goto command service called, waiting for service...")
        rospy.wait_for_service('goto_command', timeout=5)
        print("service found, calling service...")
        try:
            goto_service = rospy.ServiceProxy('goto_command', goto_command)
            req = goto_commandRequest()
            req.dest_x = x
            req.dest_y = y

            res = goto_service(req)
            if not res.success:
                print("Failed to send goto command")
            self.state_refs_np = np.array(res.state_refs.data).reshape(3, -1)
            self.attributes_np = np.array(res.wp_attributes.data)
            print("Goto_command service call successful. shape: ", self.state_refs_np.shape)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    def call_set_states_service(self, x, y):
        print("set states service called, waiting for service...")
        rospy.wait_for_service('set_states', timeout=5)
        print("service found, calling service...")
        try:
            set_states_service = rospy.ServiceProxy('set_states', set_states)
            req = set_statesRequest()
            req.x = x
            req.y = y
            res = set_states_service(req)
            if not res.success:
                print("Failed to send set states command")
            print("Set_states service call successful. shape: ", self.state_refs_np.shape)
        except rospy.ServiceException as e:
            rospy.logerr(f"Service call failed: {e}")
    def call_start_service(self, start):
        print("service call")
        rospy.wait_for_service("/start_bool", timeout=5)
        try:
            # Create a service proxy
            service_proxy = rospy.ServiceProxy("/start_bool", SetBool)
            request = SetBoolRequest()
            request.data = start
            # Call the service
            response = service_proxy(request)
            if response.success:
                rospy.loginfo("Service call succeeded!")
            else:
                rospy.logerr("Service call failed!")
        except rospy.ServiceException as e:
            print("Service call failed:", e)
            
    # ROS callback functions
    def lane_callback(self, msg):
        self.center = msg.center
        self.crosswalk = msg.crosswalk
        self.stopline = msg.stopline
    def sign_callback(self, sign):
        if sign.data:
            self.numObj = len(sign.data) // 10
            if self.numObj > 0:
                self.detected_objects = np.array(sign.data)#.reshape(-1, 7).T
        else:
            self.numObj = 0
    def waypoint_callback(self, data):
        self.waypoints = data.data
    def road_objects_callback(self, msg):
        self.detected_data = np.array(msg.data).reshape(-1, self.road_msg_length)
    def add_lane_detection_to_image(self, image):
        if self.center is None:
            return image
        # Draw the center line
        cv2.line(image, (int(self.center), image.shape[0]), (int(self.center), int(0.8 * image.shape[0])), (0, 0, 255), 5)

        # Add text if stopline or crosswalk is detected
        if self.stopline > 0:
            cv2.putText(image, "Stopline detected!", 
                        (int(image.shape[1] * 0.5), int(image.shape[0] * 0.3)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2, cv2.LINE_AA)

        if self.crosswalk:
            cv2.putText(image, "Crosswalk detected!", 
                        (int(image.shape[1] * 0.5), int(image.shape[0] * 0.4)), 
                        cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2, cv2.LINE_AA)
        return image

    def add_sign_detection_to_image(self, image):
        for i in range(self.numObj):
            try:
                id = int(self.detected_objects[10 * i + 6])
            except:
                return
            if self.detected_objects[10 * i + 5] < self.confidence_thresholds[id]:
                continue

            color_index = id % len(self.COLOR_LIST)
            color = tuple(int(c * 255) for c in self.COLOR_LIST[color_index])  # Scale color to [0, 255]

            mean_color = np.mean(color)
            text_color = (0, 0, 0) if mean_color > 127 else (255, 255, 255)

            confidence = self.detected_objects[10 * i + 5] * 100
            distance = self.detected_objects[10 * i + 4]
            text = f"{self.class_names[id]} {confidence:.1f}% {distance:.2f}m"

            label_size, baseLine = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)

            x1 = int(self.detected_objects[10 * i])
            y1 = int(self.detected_objects[10 * i + 1])
            x2 = int(self.detected_objects[10 * i + 2])
            y2 = int(self.detected_objects[10 * i + 3])

            cv2.rectangle(image, (x1, y1), (x2, y2), color, 2, lineType=cv2.LINE_AA)

            y = y1 - label_size[1] - baseLine
            if y < 0:
                y = 0
            x = x1
            if x + label_size[0] > image.shape[1]:
                x = image.shape[1] - label_size[0]
            
            txt_bk_color = tuple(int(c * 0.7) for c in color)
            cv2.rectangle(image, (x, y), (x + label_size[0], y + label_size[1] + baseLine), txt_bk_color, -1)

            cv2.putText(image, text, (x, y + label_size[1]), cv2.FONT_HERSHEY_SIMPLEX, 0.5, text_color, 1)

        return image
    def camera_callback(self, msg):
        if self.show_depth:
            return
        cv_image = self.bridge.imgmsg_to_cv2(msg, "bgr8")
        cv_image = self.add_sign_detection_to_image(cv_image)
        cv_image = self.add_lane_detection_to_image(cv_image)
        rgb_image = cv2.cvtColor(cv_image, cv2.COLOR_BGR2RGB)
        rgb_image = cv2.resize(rgb_image, (self.camera_w, self.camera_h))
        h, w, ch = rgb_image.shape
        bytes_per_line = ch * w
        qt_image = QImage(rgb_image.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.camera_label.setPixmap(pixmap)
    def depth_callback(self, msg):
        if not self.show_depth:
            return
        depth_image = self.bridge.imgmsg_to_cv2(msg, "32FC1")
        # depth_image = cv2.resize(depth_image, (self.camera_w, self.camera_h))
        # Apply normalization with a focus on closer objects
        depth_normalized = cv2.normalize(depth_image, None, 50, 255, cv2.NORM_MINMAX)
        depth_colored = cv2.applyColorMap(depth_normalized.astype(np.uint8), cv2.COLORMAP_TURBO)  # TURBO colormap for better contrast
        depth_colored = self.add_sign_detection_to_image(depth_colored)
        depth_colored = self.add_lane_detection_to_image(depth_colored)
        depth_colored = cv2.resize(depth_colored, (self.camera_w, self.camera_h))
        h, w, ch = depth_colored.shape
        bytes_per_line = ch * w
        qt_image = QImage(depth_colored.data, w, h, bytes_per_line, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qt_image)
        self.camera_label.setPixmap(pixmap)
    def message_callback(self, msg):
        self.message_history.append(msg.data)
        if len(self.message_history) > 6:
            self.message_history.pop(0)
    
    def update_stopwatch(self):
        self.centiseconds += 1
        minutes = (self.centiseconds // 6000) % 60
        seconds = (self.centiseconds // 100) % 60
        centiseconds = self.centiseconds % 100
        self.timer_label.setText(f'{minutes:02d}:{seconds:02d}:{centiseconds:02d}')
            
    def toggle_visibility(self):
        self.show_signs = not self.show_signs

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
    
    def toggle_depth(self):
        self.show_depth = not self.show_depth
        
    def call_waypoint_service(self, vref_name, path_name, x0, y0, yaw0):
        rospy.wait_for_service('waypoint_path', timeout=5)
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
                for i in range(0, len(self.waypoints)-1, 8):
                    center = (int(self.waypoints[i]/20.696*image.shape[1]),int((13.786-self.waypoints[i+1])/13.786*image.shape[0]))
                    cv2.circle(image, center, radius=1, color=(0, 255, 255), thickness=-1)
            if self.detected_data is None:
                return
            x = self.detected_data[0, self.road_msg_dict['x']]
            y = self.detected_data[0, self.road_msg_dict['y']]
            yaw = self.detected_data[0, self.road_msg_dict['orientation']]
            z = self.detected_data[0, self.road_msg_dict['z']]
            self.position_label.setText(f'Position: (x: {x:.2f}, y: {y:.2f}, yaw: {yaw:.2f}, z: {z:.2f})')
            speed = self.detected_data[0, self.road_msg_dict['speed']]
            self.speed_label.setText(f'Speed: {speed:.2f} m/s')
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
        LENGTH = 4.5*0.108 *800*self.scale_factor/20.696 # Car length
        WIDTH = 2.0*0.108  *800*self.scale_factor/20.696 # Car width
        BACKTOWHEEL = 1.0*0.108 *800*self.scale_factor/20.696 # Distance from back to the wheel
        WHEEL_LEN = 0.6*0.108 *800*self.scale_factor/20.696 # Length of the wheel
        WHEEL_WIDTH = 0.2*0.108 *800*self.scale_factor/20.696 # Width of the wheel
        TREAD = 0.7*0.108 *800*self.scale_factor/20.696 # Distance between left and right wheels
        WB = 0.27 *800*self.scale_factor/20.696 # Wheelbase: distance between the front and rear wheels
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

        cv2.circle(image, (int(x), int(y)), int(WIDTH/2.5), (143, 28, 90), -1)
        
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
            
    def update_map(self):
        display_image = self.map_image.copy()

        if self.show_path:
            display_image = self.illustrate_path(display_image)

        self.draw_objects(display_image)
        self.draw_detected_objects(display_image)
        self.message_display.setPlainText("\n".join(self.message_history))
        
        display_image = cv2.cvtColor(display_image, cv2.COLOR_BGR2RGB)
        height, width, channel = display_image.shape
        step = channel * width
        q_img = QImage(display_image.data, width, height, step, QImage.Format_RGB888)
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
    
    def mousePressEvent(self, event):
        # Get the position of the mouse click in the scene coordinates
        scene_position = self.graphics_view.mapToScene(event.pos())

        # Get the pixel coordinates relative to the image
        image_x = scene_position.x() - 25
        image_y = scene_position.y() - 23

        # Remove any previous marker
        if hasattr(self, 'cursor_marker') and self.cursor_marker:
            self.scene.removeItem(self.cursor_marker)
        if hasattr(self, 'cursor_marker_x1') and self.cursor_marker_x1:
            self.scene.removeItem(self.cursor_marker_x1)
        if hasattr(self, 'cursor_marker_x2') and self.cursor_marker_x2:
            self.scene.removeItem(self.cursor_marker_x2)
            
        # Check if the click is within the bounds of the image
        if 0 <= image_x <= self.image_width and 0 <= image_y <= self.image_height:
            # Convert the pixel coordinates to real-world coordinates
            self.cursor_x = image_x * (self.image_width_real / self.image_width)
            self.cursor_y = 13.786 - image_y * (self.image_height_real / self.image_height)

            # Display the real-world coordinates in the bottom left corner (or anywhere else)
            self.cursor_label.setText(f'Cursor: (x: {self.cursor_x:.2f} m, y: {self.cursor_y:.2f} m)')
            cursor_radius = 10  # Adjust the size of the cursor
            pen = QPen(QColor(0, 150, 255), 2)
            # Create a circle to represent the cursor marker
            self.cursor_marker = self.scene.addEllipse(
                image_x - cursor_radius, image_y - cursor_radius,
                cursor_radius * 2, cursor_radius * 2, pen
            )
            marker_size = 20  # Adjust the size of the "X"
            pen = QPen(QColor(255, 0, 0), 3)
            self.cursor_marker_x1 = self.scene.addLine(
                image_x - marker_size / 2, image_y - marker_size / 2,
                image_x + marker_size / 2, image_y + marker_size / 2,
                pen
            )
            self.cursor_marker_x2 = self.scene.addLine(
                image_x - marker_size / 2, image_y + marker_size / 2,
                image_x + marker_size / 2, image_y - marker_size / 2,
                pen
            )
if __name__ == '__main__':
    app = QApplication(sys.argv)
    window = OpenCVGuiApp()
    window.show()
    sys.exit(app.exec_())