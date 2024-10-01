import numpy as np
import matplotlib.pyplot as plt
from numpy import cos, sin
import os
def draw_scene(cars, objects, save=True):
    if len(cars) == 0 and len(objects) == 0:
        print("No cars or objects to draw")
        return
    # Initialize the plot
    fig, ax = plt.subplots()
    ax.set_aspect('equal')

    # Draw each car
    for car_state in cars:
        draw_car(ax, car_state)
        annotate_state(ax, car_state[:2], 'Car: ({:.2f}, {:.2f}, {:.2f})'.format(*car_state))
        
    # Draw each object and annotate with its position
    for obj in objects:
        draw_object(ax, obj['pose'], obj['type'])
        annotate_state(ax, obj['pose'], '{}: ({:.2f}, {:.2f})'.format(obj['type'], *obj['pose']))

    # Determine the plot limits
    all_x_positions = [car_state[0] for car_state in cars] + [obj['pose'][0] for obj in objects]
    all_y_positions = [car_state[1] for car_state in cars] + [obj['pose'][1] for obj in objects]
    ax.set_xlim(min(all_x_positions) - 1, max(all_x_positions) + 1)
    ax.set_ylim(min(all_y_positions) - 1, max(all_y_positions) + 1)

    # Set the labels and title
    ax.set_xlabel('X world coordinate')
    ax.set_ylabel('Y world coordinate')
    ax.set_title('Vehicle and Object Position in World Frame')
    
    if save:
        path = os.path.join(os.path.dirname(__file__), 'plots')
        os.makedirs(path, exist_ok=True)
        name = 'scene.png'
        full_path = os.path.join(path, name)
        print(f"Saving scene to {full_path}")
        
        # Check if file exists
        i = 1
        while os.path.isfile(full_path):
            print(f"File {full_path} exists")
            name = f'scene{i}.png'  # Update the name variable
            full_path = os.path.join(path, name)  # Update the full_path variable
            i += 1

        plt.savefig(full_path)  # Use the updated full_path
        print(f"Scene saved as {full_path}")
    plt.grid(True)
    plt.show()


# Function to draw a car with orientation
def draw_car(ax, position, length=0.5):
    car_x, car_y, yaw = position
    dx = length * np.cos(yaw)
    dy = length * np.sin(yaw)
    ax.plot(car_x, car_y, 'bo', markersize=10)  # Car's position as a blue dot
    ax.arrow(car_x, car_y, dx, dy, head_width=0.2, head_length=0.2, fc='blue', ec='blue')

# Function to draw the detected object
def draw_object(ax, position, name):
    obj_x, obj_y = position
    ax.plot(obj_x, obj_y, 'r*', markersize=15) 
    # ax.text(obj_x + 0.1, obj_y + 0.1, name, fontsize=9, verticalalignment='bottom')

def annotate_state(ax, position, text):
    ax.text(position[0]+0.2, position[1]-0.2, text, fontsize=10, bbox=dict(facecolor='white', alpha=0.5, edgecolor='none'))

def estimate_object_pose2d(vehicle_state, bounding_box, object_distance, camera_params):
    # Unpack the vehicle state and camera parameters
    vehicle_pos = np.array(vehicle_state[:2])  # Only x and y are needed for 2D
    yaw = vehicle_state[2]
    
    # Intrinsic parameters
    fx = camera_params['fx']
    cx = camera_params['cx']
    
    # Calculate center of bounding box along the x-axis
    bbox_center_x = (bounding_box[0] + bounding_box[2]) / 2
    
    # Transform pixel coordinates to camera coordinates along the x-axis
    X_c = (bbox_center_x - cx) / fx * object_distance
    
    # Vehicle coordinates (X_v is forward, Y_v is left/right from the vehicle's perspective)
    X_v = object_distance
    Y_v = -X_c  # Negate because a positive X_c means the object is to the right in the image

    # Create transformation matrix
    rotation_matrix = np.array([[cos(yaw), -sin(yaw)],
                                [sin(yaw),  cos(yaw)]])
    
    # Transform to vehicle coordinates
    vehicle_coordinates = np.dot(rotation_matrix, np.array([X_v, Y_v]))
    
    # Transform to world coordinates
    world_coordinates = vehicle_pos + vehicle_coordinates
    
    return world_coordinates

if __name__ == "__main__":
    # Example usage
    # vehicle_state = (5, 5, np.pi/4*0) 
    vehicle_state1 = np.array([3, 5, np.pi/2])  # x, y, yaw
    vehicle_state = np.array([0.82, 0.093, 1.57])  # x, y, yaw
    # bounding_box = (300-200, 460, 340-200, 500) # (xmin, ymin, xmax, ymax)
    bounding_box = np.array([200, 480, 240, 500])  # xmin, ymin, xmax, ymax
    # i want bounding box to be: x center, y center, width, height
    xmin = 200 
    xmax = 240
    ymin = 480
    ymax = 500
    x_center = (xmin + xmax) / 2
    y_center = (ymin + ymax) / 2
    width = xmax - xmin
    height = ymax - ymin
    bounding_box = np.array([x_center, y_center, width, height])
    #541 229 97 235
    bounding_box = np.array([542, 226, 639, 466]) # xmin, ymin, xmax, ymax

    object_distance = 3 # Distance to the object in meters
    object_distance = 0.521 # Distance to the object in meters
    # camera_params = {'fx': 640, 'fy': 480, 'cx': 320, 'cy': 240} 
    camera_params = {'fx': 554.3826904296875, 'fy': 554.3826904296875, 'cx': 320, 'cy': 240} 
    camera_pose = {'x': 0, 'y': 0, 'z': 0.2} # Camera pose relative to the car

    object_world_pose2 = estimate_object_pose2d(vehicle_state, bounding_box, object_distance, camera_params)
    print(f"vehicle_state: {vehicle_state}")
    print(f"Estimated world coordinates of the object: {object_world_pose2}")
    #ground: 1.132, 0.597
    objects = [{'type': 'traffic cone', 'pose': object_world_pose2}]
    cars = [vehicle_state, vehicle_state1]

    draw_scene(cars, objects)