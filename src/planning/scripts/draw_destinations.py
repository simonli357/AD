#!/usr/bin/env python3
import os
import networkx as nx
import yaml
import cv2
import numpy as np

def main():
    current_dir = os.path.dirname(os.path.realpath(__file__))
    
    # --- 1. Load destination nodes from YAML ---
    destinations_file = os.path.join(current_dir, 'config', 'destinations_mod.yaml')
    with open(destinations_file, 'r') as f:
        destination_nodes = yaml.safe_load(f)
    # Ensure node IDs are strings (matching the graphml keys)
    destination_nodes = [str(node) for node in destination_nodes]
    
    # --- 2. Load graphml to get node positions ---
    graphml_file = os.path.join(current_dir, 'maps', 'Competition_track_graph_modified_new.graphml')
    G = nx.read_graphml(graphml_file)
    
    pos = {}
    for node, data in G.nodes(data=True):
        x = float(data.get('x', 0.0))
        y = float(data.get('y', 0.0))
        # Adjust y so that 0 is at the top (flip with respect to map height)
        pos[node] = (x, 13.786 - y)
    
    # --- 3. Load the map image ---
    map_image_file = os.path.join(current_dir, 'maps', 'Competition_track_graph_new.png')
    image = cv2.imread(map_image_file)
    if image is None:
        print(f"Error loading image from {map_image_file}")
        return
    image_height, image_width = image.shape[:2]
    
    # --- 4. Determine relative sizes for drawing ---
    # For example: circle radius is 1.5% of the image width (at least 5 pixels)
    circle_radius = max(5, int(image_width * 0.01))
    # Text parameters: font scale relative to image width
    font = cv2.FONT_HERSHEY_SIMPLEX
    font_scale = image_width / 1200.0  # Adjust as needed
    font_thickness = max(1, int(image_width / 1000.0))
    # Offsets so text doesn't overlap the circle
    text_offset_x = circle_radius-10
    text_offset_y = circle_radius-10
    
    # --- 5. Draw the circles and node numbers ---
    i  = 0
    for node in destination_nodes:
        i += 1
        if node not in pos:
            print(f"Node {node} not found in graphml.")
            continue
        x, y = pos[node]
        y = 13.786 - y
        # Convert map coordinates to pixel coordinates:
        pixel_x = int((x / 20.454) * image_width)
        pixel_y = int((y / 13.786) * image_height)
        
        # Draw a filled circle (red)
        cv2.circle(image, (pixel_x, pixel_y), circle_radius, (0, 0, 255), -1)
        
        # Overlay the node number (green text with anti-aliasing)
        cv2.putText(image, node, (pixel_x + text_offset_x, pixel_y - text_offset_y),
                    font, font_scale, (0, 255, 0), font_thickness, cv2.LINE_AA)
    
    # --- 6. Save the resulting image ---
    output_file = os.path.join(current_dir, 'maps', 'map_with_destinations.png')
    cv2.imwrite(output_file, image)
    print(f"Output image saved to {output_file}")
    print("number of nodes: ", i)

if __name__ == '__main__':
    main()
