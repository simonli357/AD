#!/usr/bin/env python3

import cv2
import pandas as pd
import numpy as np
import os
import math
import rospy
from std_msgs.msg import Float32MultiArray
from utils.srv import waypoints, waypointsRequest, waypointsResponse

state_refs_np = None
input_refs_np = None
attributes_np = None
normals_np = None
maneuver_directions_np = None

def illustrate_path(image, state_refs, attributes):
    image_copy = image.copy()
    for i in range(0, state_refs.shape[1], 8):
        radius = 2
        color = (0, 255, 255)
        if attributes[i] == 4 or attributes[i] == 5: # hard waypoints
            color = (0, 0, 255)
        if attributes[i] == 1: # crosswalk
            color = (0, 255, 0) # green
        if attributes[i] == 9: # color is red
            color = (255, 0, 0)
        if attributes[i] == 7: # color is yellow
            color = (255, 255, 0)
        if attributes[i] == 6: # color is white
            color = (255, 255, 255)
        if attributes[i] >= 100: # orange
            color = (0, 165, 255)
        if attributes[i] == 2 or attributes[i] == 102: # color is purple
            color = (255, 0, 255)
        cv2.circle(image_copy, (int(state_refs[0, i]/20.696*image.shape[1]),int((13.786-state_refs[1, i])/13.786*image.shape[0])), radius=int(radius), color=color, thickness=-1)
    # cv2.imshow('Map357', image_copy)
    # cv2.waitKey(1)
    return image_copy
    
def call_waypoint_service(vref_name, path_name, x0, y0, yaw0):
    print("waiting for service")
    rospy.wait_for_service('waypoint_path')
    print("service found")
    global state_refs_np, input_refs_np, attributes_np, normals_np, maneuver_directions_np
    try:
        waypoint_path_service = rospy.ServiceProxy('waypoint_path', waypoints)
        req = waypointsRequest()
        req.vrefName = vref_name
        req.pathName = path_name
        req.x0 = x0
        req.y0 = y0
        req.yaw0 = yaw0

        res = waypoint_path_service(req)

        state_refs_np = np.array(res.state_refs.data).reshape(-1, 3).T  # Reshape based on your path structure
        print("shape of state_refs_np", state_refs_np.shape)
        input_refs_np = np.array(res.input_refs.data).reshape(-1, 2).T  # Adjust based on input dimension
        attributes_np = np.array(res.wp_attributes.data)
        normals_np = np.array(res.wp_normals.data).reshape(-1, 2).T  # Assuming 2D normals
        # maneuver_directions_np = np.array(res.maneuver_directions.data)
        print("Service call successful.")
        return True

    except rospy.ServiceException as e:
        rospy.logerr(f"Service call failed: {e}")
        return False

current_dir = os.path.dirname(os.path.abspath(__file__))
assets_dir = os.path.join(current_dir, 'assets')
data = pd.read_csv(os.path.join(assets_dir, 'coordinates_with_context.csv'))

scale_factor = 1.5
map_width_pixels, map_height_pixels = int(800 * scale_factor), int(533 * scale_factor)  # Adjust according to your image resolution

map_image = cv2.imread(os.path.join(current_dir, 'assets/map1.png'))
map_image = cv2.resize(map_image, (map_width_pixels, map_height_pixels))

scale_x = map_width_pixels / 20.696
scale_y = map_height_pixels / 13.786

sign_size = int(20 * scale_factor)
arrow_size = int(20 * scale_factor)
intersection_size = int(20 * scale_factor)
show_elements = True
show_lanes = True
show_cars = True
show_destinations = True
show_path = True

LENGTH = 4.5*0.108 *800*scale_factor/20.696 # Car length
WIDTH = 2.0*0.108  *800*scale_factor/20.696 # Car width
BACKTOWHEEL = 1.0*0.108 *800*scale_factor/20.696 # Distance from back to the wheel
WHEEL_LEN = 0.6*0.108 *800*scale_factor/20.696 # Length of the wheel
WHEEL_WIDTH = 0.2*0.108 *800*scale_factor/20.696 # Width of the wheel
TREAD = 0.7*0.108 *800*scale_factor/20.696 # Distance between left and right wheels
WB = 0.258 *800*scale_factor/20.696 # Wheelbase: distance between the front and rear wheels

def draw_car(image, x, y, yaw, steer=0.0, car_color=(0, 255, 255), wheel_color=(0, 0, 0)):
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
    
    draw_arrow(image, x, y, -yaw, LENGTH * 1.2)
    
def draw_intersection(image, x, y, orientation, size):
    x, y = int(x), int(y)
    length = max(1, size) # Dynamic size from trackbar
    if orientation == 0 or orientation == np.pi or orientation == 2*np.pi:  # Horizontal line
        cv2.line(image, (x - length // 2, y), (x + length // 2, y), (255, 0, 0), 2)
    else:  # Vertical line
        cv2.line(image, (x, y - length // 2), (x, y + length // 2), (255, 0, 0), 2)

# Function to draw an arrow
def draw_arrow(image, x, y, orientation, size):
    x, y = int(x), int(y)
    arrow_length = max(1, size)  # Dynamic size from trackbar
    x_end = int(x + arrow_length * np.cos(orientation))
    y_end = int(y - arrow_length * np.sin(orientation))
    cv2.arrowedLine(image, (x, y), (x_end, y_end), (0, 0, 255), 2, tipLength=0.3)

def draw_lane(image, x, y, orientation):
    x, y = int(x), int(y)

    # For North-South lanes (orientation = 0 or pi)
    if orientation == 0 or orientation == 2 * np.pi:
        cv2.line(image, (0, y), (image.shape[1], y), (0, 255, 0), 2)
    elif orientation == np.pi:
        cv2.line(image, (0, y), (image.shape[1], y), (0, 0, 255), 2)
    elif orientation == np.pi/2:
        cv2.line(image, (x, 0), (x, image.shape[0]), (255, 0, 0), 2)
    elif orientation == 3*np.pi/2:
        cv2.line(image, (x, 0), (x, image.shape[0]), (0, 255, 255), 2)

# Function to draw signs using images
def draw_sign(image, x, y, sign_type, orientation, size):
    try:
        img = None
        assets_path = os.path.join(current_dir, 'assets')
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
        
        if img is not None:
            size = max(5, size)
            img = cv2.resize(img, (size, size))  # Resize sign based on trackbar value

            # Rotate the image according to orientation (optional)
            center = (img.shape[1] // 2, img.shape[0] // 2)
            # rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(orientation), 1.0)
            rotation_matrix = cv2.getRotationMatrix2D(center, np.degrees(0), 1.0)
            img = cv2.warpAffine(img, rotation_matrix, (img.shape[1], img.shape[0]))

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

# Callback function for trackbars
def update_size(val):
    global sign_size, arrow_size, intersection_size
    sign_size = cv2.getTrackbarPos('Sign Size', 'Map')
    arrow_size = cv2.getTrackbarPos('Arrow Size', 'Map')
    intersection_size = cv2.getTrackbarPos('Intersection Size', 'Map')

# Toggle visibility
def toggle_visibility():
    global show_elements
    show_elements = not show_elements
def toggle_lanes():
    global show_lanes
    show_lanes = not show_lanes
def toggle_cars():
    global show_cars
    show_cars = not show_cars
def toggle_destinations():
    global show_destinations
    show_destinations = not show_destinations
def toggle_path():
    global show_path
    show_path = not show_path
def get_next_filename(directory, base_name, extension):
    i = 1
    while os.path.exists(f"{directory}/{base_name}{i}{extension}"):
        i += 1
    return f"{directory}/{base_name}{i}{extension}"

rospy.init_node('visualizer')
vref_name = '25'
path_name = 'speedrun'
x0 = 11.8
y0 = 2.02
yaw0 = 0.0
result = call_waypoint_service(vref_name, path_name, x0, y0, yaw0)
if result:
    rospy.loginfo(f"State References:\n{state_refs_np.shape}")

cv2.namedWindow('Map')

cv2.createTrackbar('Sign Size', 'Map', sign_size, 100, update_size)
cv2.createTrackbar('Arrow Size', 'Map', arrow_size, 100, update_size)
cv2.createTrackbar('Intersection Size', 'Map', intersection_size, 100, update_size)

map_image_with_path = illustrate_path(map_image, state_refs_np, attributes_np)

rate = rospy.Rate(100)
while True:
    # Create a copy of the map to draw on
    if show_path:
        display_image = map_image_with_path.copy()
    else:
        display_image = map_image.copy()

    # Loop through data and plot each point if elements are visible
    if show_elements or show_lanes or show_cars or show_destinations:
        for index, row in data.iterrows():
            x, y, entity_type, orientation = row['X'], row['Y'], row['Type'], row['Orientation']

            # Convert map coordinates to pixel coordinates
            pixel_x = int(x * scale_x)
            pixel_y = int((map_height_pixels - y * scale_y))  # Invert y-axis
            orientation = orientation  # Invert orientation

            if entity_type == 'Intersection':
                if show_elements:
                    draw_intersection(display_image, pixel_x, pixel_y, orientation + np.pi / 2, intersection_size)
                    draw_arrow(display_image, pixel_x, pixel_y, -orientation, arrow_size)
            elif entity_type == 'Lane':
                if show_lanes:
                    draw_lane(display_image, pixel_x, pixel_y, orientation)  # Draw lane center lines
            elif entity_type == 'Car':
                if show_cars:
                    draw_car(display_image, pixel_x, pixel_y, -orientation, steer=0.0)
            elif entity_type == 'Destination':
                if show_destinations:
                    size = int(0.15 / 20.696 * 800 * scale_factor)
                    cv2.circle(display_image, (pixel_x, pixel_y), size, (235, 206, 135), -1)
            else:
                if show_elements:
                    draw_sign(display_image, pixel_x, pixel_y, entity_type, -orientation, sign_size)
                    draw_arrow(display_image, pixel_x, pixel_y, -orientation , arrow_size)

    cv2.imshow('Map', display_image)

    key = cv2.waitKey(1) & 0xFF

    if key == ord('q'):  # Press 'q' to quit
        break
    elif key == ord('t'):  # Press 't' to toggle visibility
        toggle_visibility()
    elif key == ord('l'):
        toggle_lanes()
    elif key == ord('c'):
        toggle_cars()
    elif key == ord('d'):
        toggle_destinations()
    elif key == ord('p'):
        toggle_path()
    elif key == ord('s'):  # Press 's' to save the image
        filename = get_next_filename('assets', '', '.png')
        cv2.imwrite(filename, display_image)
        print(f"Image saved as {filename}")
    rate.sleep()

cv2.destroyAllWindows()
