import cv2
import numpy as np

# Constants
map_path = "/home/simonli/mpc/scripts/maps/Track.png"
map_width_px = 9766
map_height_px = 6502
map_width_m = 20.696
map_height_m = 13.786
pole_size = 0.0514
INNER_LANE_OFFSET = 0.5  # Example value, you need to replace it with the actual value

# Coordinate conversion factor
x_factor = map_width_px / map_width_m
y_factor = map_height_px / map_height_m

# Calculate the offset
ofs6 = INNER_LANE_OFFSET / 2 - pole_size / 2
hsw = pole_size / 2

# Coordinates calculation functions
def real_to_pixel(coords):
    return (int(coords[0] * x_factor), int(map_height_px - coords[1] * y_factor))

# Parking coordinates
PARKING_SPOT_RIGHT = [9.07365, 0.703 - INNER_LANE_OFFSET / 2 + pole_size / 2]
PARKING_SPOT_LEFT = [9.07365, 1.1527 + INNER_LANE_OFFSET / 2 - pole_size / 2]

# Lane centers
NORTH_FACING_LANE_CENTERS = [0.579612 + ofs6, 2.744851 + ofs6, 4.9887 + ofs6, 6.77784 + ofs6, 6.8507 + ofs6, 16.954 + ofs6, 15.532 + ofs6, 16.1035 + ofs6]
SOUTH_FACING_LANE_CENTERS = [0.50684 - ofs6, 2.667 - ofs6, 4.9156 - ofs6, 15.165279 + ofs6, 15.1632 + ofs6]
EAST_FACING_LANE_CENTERS = [12.904 + ofs6, 10.5538 - ofs6, 0.503891 - ofs6, 3.79216 - ofs6, 6.6816 - ofs6, 10.5538 - ofs6]
WEST_FACING_LANE_CENTERS = [13.314624 + ofs6, 10.633 + ofs6, 3.86375 + ofs6, 0.58153 + ofs6, 3.8661 + ofs6, 6.753 + ofs6, 13.278 + ofs6]

# Load map image
map_image = cv2.imread(map_path)

# Function to resize image
def resize_image(image, scale_percent):
    width = int(image.shape[1] * scale_percent / 100)
    height = int(image.shape[0] * scale_percent / 100)
    return cv2.resize(image, (width, height), interpolation = cv2.INTER_AREA)

# Resize the image to 30% of its original size
scale_percent = 15
map_image_resized = resize_image(map_image, scale_percent)

# Adjust factor for resizing
x_factor_resized = x_factor * scale_percent / 100
y_factor_resized = y_factor * scale_percent / 100

# Convert real world coordinates to resized pixel coordinates
def real_to_pixel_resized(coords):
    return (int(coords[0] * x_factor_resized), int(map_image_resized.shape[0] - coords[1] * y_factor_resized))

# Draw points on the map
def draw_point(image, coords, color, label):
    pixel_coords = real_to_pixel_resized(coords)
    cv2.circle(image, pixel_coords, 10, color, -1)
    cv2.putText(image, label, (pixel_coords[0] + 10, pixel_coords[1] + 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, color, 2)

# Draw parking spots
draw_point(map_image_resized, PARKING_SPOT_RIGHT, (0, 255, 0), 'Right Parking Spot')
draw_point(map_image_resized, PARKING_SPOT_LEFT, (0, 255, 0), 'Left Parking Spot')

# Draw lane centers
for i, lane in enumerate(NORTH_FACING_LANE_CENTERS):
    draw_point(map_image_resized, [lane, 0], (255, 0, 0), f'North {i}')

for i, lane in enumerate(SOUTH_FACING_LANE_CENTERS):
    draw_point(map_image_resized, [lane, 0], (0, 0, 255), f'South {i}')

for i, lane in enumerate(EAST_FACING_LANE_CENTERS):
    draw_point(map_image_resized, [0, lane], (255, 255, 0), f'East {i}')

for i, lane in enumerate(WEST_FACING_LANE_CENTERS):
    draw_point(map_image_resized, [0, lane], (0, 255, 255), f'West {i}')

# Display the resized map with points
cv2.imshow('Map with Points', map_image_resized)
cv2.waitKey(0)
cv2.destroyAllWindows()
