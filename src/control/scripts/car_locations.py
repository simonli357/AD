#!/usr/bin/env python3

import rospy
from std_msgs.msg import Float64MultiArray
import cv2
import numpy as np
import os

# Map and Car Icon dimensions
MAP_WIDTH_METERS = 20.696
MAP_HEIGHT_METERS = 13.786

# Load map image
current_dir = os.path.dirname(os.path.abspath(__file__))
map_path = os.path.join(current_dir, 'maps', 'Competition_track_graph.png')
base_img = cv2.imread(map_path)
IMG_HEIGHT, IMG_WIDTH, _ = base_img.shape

# Load car icon image (Assuming it's a PNG with transparency)
car_icon_path = os.path.join(current_dir, 'maps', 'car_icon.png')
car_icon = cv2.imread(car_icon_path)

# Ensure car icon is in RGB format
if len(car_icon.shape) == 2:
    # Convert grayscale to RGB
    car_icon = cv2.cvtColor(car_icon, cv2.COLOR_GRAY2RGB)

# Ensure the car icon has the same number of channels as the map image
if car_icon.shape[2] != base_img.shape[2]:
    print("Error: Number of channels in car icon does not match the map image.")
    exit(1)

# Resize the car icon if it's too large
ICON_SIZE = (100, 100)  # Example size, adjust as needed
car_icon = cv2.resize(car_icon, ICON_SIZE)
print(car_icon.shape)

def world_to_pixel(world_x, world_y, img_width, img_height):
    pixel_x = int((world_x / MAP_WIDTH_METERS) * img_width)
    pixel_y = int(img_height - (world_y / MAP_HEIGHT_METERS) * img_height)
    return pixel_x, pixel_y

def overlay_icon(background, icon, position):
    x, y = position
    h, w = icon.shape[:2]

    # Adjust position to draw the icon centered at position
    x = x - w // 2
    y = y - h // 2

    # Handle corners (to avoid out of bounds indexing)
    if x < 0 or y < 0 or x + w > background.shape[1] or y + h > background.shape[0]:
        return  # Skip if the icon goes out of the image bounds

    # Overlay the icon
    background[y:y+h, x:x+w] = icon

def callback(data):
    car_positions = np.array(data.data)
    car_positions = car_positions.reshape(-1, 2)

    map_image = base_img.copy()
    for pos in car_positions:
        pixel_x, pixel_y = world_to_pixel(pos[0], pos[1], IMG_WIDTH, IMG_HEIGHT)
        overlay_icon(map_image, car_icon, (pixel_x, pixel_y))

    resized_image = cv2.resize(map_image, (800, 600))
    cv2.imshow("Car Locations", resized_image)
    cv2.waitKey(1)

def listener():
    rospy.init_node('car_location_listener', anonymous=True)
    rospy.Subscriber('car_locations', Float64MultiArray, callback)

    rospy.spin()

if __name__ == '__main__':
    listener()
