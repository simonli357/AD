import pandas as pd
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrow
from matplotlib.transforms import Affine2D
import matplotlib.image as mpimg
import numpy as np

# Load data from CSV
data = pd.read_csv('assets/coordinates_with_context.csv')

# Set map size
map_width, map_height = 20.696, 13.786

# Load the map image
map_image = mpimg.imread('assets/map1.png')

# Function to draw intersections with perpendicular lines based on orientation
def draw_intersection(ax, x, y, orientation):
    # Line length for the intersection
    length = 0.5
    if orientation == 0 or orientation == np.pi:  # Horizontal line
        ax.plot([x - length / 2, x + length / 2], [y, y], color='blue', lw=2)
    else:  # Vertical line
        ax.plot([x, x], [y - length / 2, y + length / 2], color='blue', lw=2)

def draw_arrow(ax, x, y, orientation):
    arrow_length = 0.5  # Length of the arrow
    dx = arrow_length * np.cos(orientation)  # Change in x based on orientation
    dy = arrow_length * np.sin(orientation)  # Change in y based on orientation
    ax.arrow(x, y, dx, dy, head_width=0.1, head_length=0.1, fc='red', ec='red')

# Function to draw signs using images, scaling and rotating them
def draw_sign(ax, x, y, sign_type, orientation):
    try:
        img = None
        if sign_type == 'Roundabout':
            img = mpimg.imread('assets/roundabout.jpg')
        elif sign_type == 'Sign':
            img = mpimg.imread('assets/stopsign.jpg')
        elif sign_type == 'Parking':
            img = mpimg.imread('assets/parking.jpg')
        elif sign_type == 'Crosswalk':
            img = mpimg.imread('assets/crosswalk.jpg')
        if img is not None:
            imagebox = ax.imshow(img, extent=(x - 0.12, x + 0.12, y - 0.12, y + 0.12))

            # Rotate the image according to orientation
            transform = Affine2D().rotate_around(x, y, orientation) + ax.transData
            imagebox.set_transform(transform)
    except FileNotFoundError:
        print(f"Image for {sign_type} not found. Skipping.")

# Create plot
fig, ax = plt.subplots(figsize=(10, 7))

ax.imshow(map_image, extent=[0, map_width, 0, map_height])
ax.set_xlim(0, map_width)
ax.set_ylim(0, map_height)
ax.set_aspect('equal')

# Loop through data and plot each point
for index, row in data.iterrows():
    x, y, entity_type, orientation = row['X'], row['Y'], row['Type'], row['Orientation']

    if entity_type == 'Intersection':
        draw_intersection(ax, x, y, orientation+np.pi/2)
    else:
        draw_sign(ax, x, y, entity_type, orientation)
    draw_arrow(ax, x, y, orientation + np.pi) 
    

# Set labels and grid
ax.set_title("Map with Intersections and Signs")
ax.set_xlabel("X")
ax.set_ylabel("Y")
ax.grid(True)

# Show plot
plt.show()
