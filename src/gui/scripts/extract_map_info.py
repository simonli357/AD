
import math
import pandas as pd
import yaml
import os
import networkx as nx

# Constants
LANE_OFFSET = 0.3935
INNER_LANE_OFFSET = 0.3465
pole_size = 0.0514
ofs6 = INNER_LANE_OFFSET / 2 - pole_size / 2
hsw = pole_size / 2
sign_ofs1 = 0.056
sign_ofs2 = 0.051

# Function to store coordinates with context
def store_coordinates_with_context(coords, orientation):
    data = []
    for coord in coords:
        data.append([coord[0], coord[1], coord[2], orientation])
    return data
# static const std::vector<double> NORTH_FACING_LANE_CENTERS = {0.579612+ofs6, 2.744851+ofs6, 4.9887+ofs6, 6.77784+ofs6, 6.8507+ofs6, 16.954+ofs6, 15.532+ofs6, 16.1035+ofs6};
#     static const std::vector<double> SOUTH_FACING_LANE_CENTERS = {0.50684-ofs6, 2.667-ofs6, 4.9156-ofs6, 15.165279+ofs6, 15.1632+ofs6};
#     // add half of inner lane width to the y values
#     // static constexpr std::array<double, 13> X_ALIGNED_LANE_CENTERS = {13.314624, 12.94356, 10.669, 10.2963, 3.89, 0.598716, 0.9698, 3.516515, 3.88667, 6.4122, 6.78514, 11.6955, 12.0661};
#     static const std::vector<double> EAST_FACING_LANE_CENTERS = {12.904+ofs6, 10.5538-ofs6, 0.503891-ofs6, 3.79216-ofs6, 6.6816-ofs6, 10.5538-ofs6};
#     static const std::vector<double> WEST_FACING_LANE_CENTERS = {13.314624+ofs6, 10.633+ofs6, 3.86375+ofs6, 0.58153+ofs6, 3.8661+ofs6, 6.753+ofs6, 13.278+ofs6};

north_facing_lane_centers = [
    [0.579612 + ofs6, 0, "Lane"],
    [2.744851 + ofs6, 0, "Lane"],
    [4.9887 + ofs6, 0, "Lane"],
    [6.51 + ofs6, 0, "Lane"],
    [6.8507 + ofs6, 0, "Lane"],
    [16.954 + ofs6, 0, "Lane"],
    [15.532 + ofs6, 0, "Lane"],
    [16.1035 + ofs6, 0, "Lane"]
]

south_facing_lane_centers = [
    [0.50684 - ofs6, 0, "Lane"],
    [2.667 - ofs6, 0, "Lane"],
    [4.9156 - ofs6, 0, "Lane"],
    [15.165279 + ofs6, 0, "Lane"],
    [15.1632 + ofs6, 0, "Lane"],
    [15.7356 + ofs6, 0, "Lane"]
]

east_facing_lane_centers = [
    [0, 12.904 + ofs6, "Lane"],
    [0, 10.5538 - ofs6, "Lane"],
    [0, 0.503891 - ofs6, "Lane"],
    [0, 1.072 - ofs6, "Lane"],
    [0, 3.79216 - ofs6, "Lane"],
    [0, 6.6816 - ofs6, "Lane"],
    [0, 10.5538 - ofs6, "Lane"],
    [0, 11.6588 + ofs6, "Lane"]
]

west_facing_lane_centers = [
    [0, 13.314624 + ofs6, "Lane"],
    [0, 10.633 + ofs6, "Lane"],
    [0, 3.86375 + ofs6,"Lane"],
    [0, 0.58153 + ofs6, "Lane"],
    [0, 3.8661 + ofs6, "Lane"],
    [0, 6.753 + ofs6, "Lane"],
    [0, 13.278 + ofs6, "Lane"],
    [0, 12.032 + ofs6, "Lane"]
]

south_facing_intersections = [
    [16.075438 - ofs6, 11.684077 - hsw, "Intersection", "SOUTH"],
    [15.46 - ofs6, 4.58 - hsw, "Intersection", "SOUTH"],
    [15.165376 + ofs6, 1.4972 - hsw, "Intersection", "SOUTH"],
    [4.9156 - ofs6, 4.581664 - hsw, "Intersection", "SOUTH"],
    [4.9143 - ofs6, 1.3 - hsw, "Intersection", "SOUTH"],
    [2.667 - ofs6, 1.3 - hsw, "Intersection", "SOUTH"],
    [2.67 - ofs6, 4.584 - hsw, "Intersection", "SOUTH"],
    [0.50684 - ofs6, 4.5849 - hsw, "Intersection", "SOUTH"],
    [0.50684 - ofs6, 7.470675 - hsw, "Intersection", "SOUTH"],
    [0.50684 - ofs6, 10.584 - hsw, "Intersection", "SOUTH"]
]

north_facing_intersections = [
    [0.579612 + ofs6, 9.0727 + hsw, "Intersection", "NORTH"],
    [0.579612 + ofs6, 5.96247 + hsw, "Intersection", "NORTH"],
    [0.579612 + ofs6, 3.07145 + hsw, "Intersection", "NORTH"],
    [2.744851 + ofs6, 3.07145 + hsw, "Intersection", "NORTH"],
    [2.744851 + ofs6, 5.9603 + hsw, "Intersection", "NORTH"],
    [4.9887 + ofs6, 5.958 + hsw, "Intersection", "NORTH"],
    [4.9887 + ofs6, 3.07 + hsw, "Intersection", "NORTH"],
    [6.77784 - ofs6, 3.44261 + hsw, "Intersection", "NORTH"],
    [16.104 + ofs6, 9.5053 + hsw, "Intersection", "NORTH"]
]

west_facing_intersections = [
    [17.1571 - hsw, 10.633 + ofs6, "Intersection", "WEST"],
    [1.296 - hsw, 3.86375 + ofs6, "Intersection", "WEST"],
    [3.4543 - hsw, 3.865637 + ofs6, "Intersection", "WEST"],
    [5.71 - hsw, 3.8661 + ofs6, "Intersection", "WEST"],
    [5.708 - hsw, 6.753 + ofs6, "Intersection", "WEST"],
    [3.4547 - hsw, 6.7545 + ofs6, "Intersection", "WEST"],
    [1.296 - hsw, 6.754754 + ofs6, "Intersection", "WEST"],
    [7.568552 - hsw, 3.8674 + ofs6, "Intersection", "WEST"],
    [3.45624 - hsw, 0.58153 + ofs6, "Intersection", "WEST"],
    [16.2485 - hsw, 3.8678 + ofs6, "Intersection", "WEST"]
]

east_facing_intersections = [
    [1.95075 + hsw, 0.503891 - ofs6, "Intersection", "EAST"],
    [1.95075 + hsw, 3.794 - ofs6, "Intersection", "EAST"],
    [1.95 + hsw, 6.6816 - ofs6, "Intersection", "EAST"],
    [4.19476 + hsw, 6.681 - ofs6, "Intersection", "EAST"],
    [4.19476 + hsw, 3.79216 - ofs6, "Intersection", "EAST"],
    # [6.0735 + hsw, 0.5949 + ofs6, "Intersection", "EAST"],
    [4.194644 + hsw, 0.503836 - ofs6, "Intersection", "EAST"],
    [14.7386 + hsw, 1.07135 - ofs6, "Intersection", "EAST"],
    [14.983 + hsw, 10.5538 - ofs6, "Intersection", "EAST"]
]

south_facing_signs = [
    [15.46 - ofs6 * 2 - pole_size - sign_ofs1, 4.58 + sign_ofs2, "Sign", "SOUTH"],
    [15.165376 - pole_size - sign_ofs1, 1.4972 + sign_ofs2, "Sign", "SOUTH"],
    [4.9156 - ofs6 * 2 - pole_size - sign_ofs1, 4.581664 + sign_ofs2, "Sign", "SOUTH"],
    [4.9143 - ofs6 * 2 - pole_size - sign_ofs1, 1.3 + sign_ofs2, "Sign", "SOUTH"],
    [2.667 - ofs6 * 2 - pole_size - sign_ofs1, 1.3 + sign_ofs2, "Sign", "SOUTH"],
    [0.50684 - ofs6 * 2 - pole_size - sign_ofs1, 4.5849 + sign_ofs2, "Sign", "SOUTH"],
    [0.50684 - ofs6 * 2 - pole_size - sign_ofs1, 7.470675 + sign_ofs2, "Sign", "SOUTH"],
    [0.50684 - ofs6 * 2 - pole_size - sign_ofs1, 10.584 + sign_ofs2, "Sign", "SOUTH"]
]

north_facing_signs = [
    [0.579612 + ofs6 * 2 + pole_size + sign_ofs1, 9.0727 - sign_ofs2, "Sign", "NORTH"],
    [0.579612 + ofs6 * 2 + pole_size + sign_ofs1, 5.96247 - sign_ofs2, "Sign", "NORTH"],
    [0.579612 + ofs6 * 2 + pole_size + sign_ofs1, 3.07145 - sign_ofs2, "Sign", "NORTH"],
    [2.744851 + ofs6 * 2 + pole_size + sign_ofs1, 5.9603 - sign_ofs2, "Sign", "NORTH"],
    [4.9887 + ofs6 * 2 + pole_size + sign_ofs1, 5.958 - sign_ofs2, "Sign", "NORTH"],
    [4.9887 + ofs6 * 2 + pole_size + sign_ofs1, 3.07 - sign_ofs2, "Sign", "NORTH"],
    [6.4836 + ofs6 * 2 + pole_size + sign_ofs1, 3.44261 - sign_ofs2, "Sign", "NORTH"]
    # [6.9, 3.425, "Sign", "NORTH"]
]

west_facing_signs = [
    # [17.1571 + sign_ofs2, 10.633 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [1.296 + sign_ofs2, 3.86375 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [5.71 + sign_ofs2, 3.8661 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [5.708 + sign_ofs2, 6.753 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [3.4547 + sign_ofs2, 6.7545 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [1.296 + sign_ofs2, 6.754754 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [7.568552 + sign_ofs2, 3.8674 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [3.45624 + sign_ofs2, 0.58153 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"],
    [16.2485 + sign_ofs2, 3.8678 + ofs6 * 2 + pole_size + sign_ofs1, "Sign", "WEST"]
]

east_facing_signs = [
    [1.95075 - sign_ofs2, 0.503891 - ofs6 * 2 - pole_size - sign_ofs1, "Sign", "EAST"],
    [1.95 - sign_ofs2, 6.6816 - ofs6 * 2 - pole_size - sign_ofs1, "Sign", "EAST"],
    [4.19476 - sign_ofs2, 6.681 - ofs6 * 2 - pole_size - sign_ofs1, "Sign", "EAST"],
    [4.19476 - sign_ofs2, 3.79216 - ofs6 * 2 - pole_size - sign_ofs1, "Sign", "EAST"],
    [4.194644 - sign_ofs2, 0.503836 - ofs6 * 2 - pole_size - sign_ofs1, "Sign", "EAST"],
    [14.7386 - sign_ofs2, 1.07135 - ofs6 * 2 - pole_size - sign_ofs1, "Sign", "EAST"],
    # [14.983 - sign_ofs2, 10.5538 - ofs6 * 2 - pole_size - sign_ofs1, "Sign", "EAST"]
]

south_facing_lights = [
    [2.67 - ofs6 * 2 - pole_size - sign_ofs1, 4.584 + sign_ofs2, "Light", "SOUTH"],
]
north_facing_lights = [
    [2.744851 + ofs6 * 2 + pole_size + sign_ofs1, 3.07145 - sign_ofs2, "Light", "NORTH"],
]
west_facing_lights = [
    [3.4543 + sign_ofs2, 3.865637 + ofs6 * 2 + pole_size + sign_ofs1, "Light", "WEST"],
]
east_facing_lights = [
    [1.95075 - sign_ofs2, 3.794 - ofs6 * 2 - pole_size - sign_ofs1, "Light", "EAST"],
]

park_ofs1_left = 0.009
park_ofs1_right = 0.016
park_ofs2 = 0.05325
parking_signs = [
    [8.99 - park_ofs2, 0.703367 - park_ofs1_right, "Parking", "WEST"],
    [8.99 - park_ofs2, 1.1522 + park_ofs1_left, "Parking", "WEST"]
]

# static constexpr double rdb_ofs1 = 0.107834;
#     static constexpr double rdb_ofs2 = 0.05361;
#     static const std::vector<std::vector<double>> ROUNDABOUT_POSES = {{{{14.9777, 10.263}}, {{16.3974, 9.455325}}, {{17.247, 11.067}}, {{15.639, 11.80325}}}};
#     static const std::vector<std::vector<double>> EAST_FACING_ROUNDABOUT = {{{14.9777-rdb_ofs2, 10.263-rdb_ofs1}}};
#     static const std::vector<std::vector<double>> NORTH_FACING_ROUNDABOUT = {{{16.4+rdb_ofs1, 9.52-rdb_ofs2}}};
#     static const std::vector<std::vector<double>> WEST_FACING_ROUNDABOUT = {{{17.164+rdb_ofs2, 10.928+rdb_ofs1}}};
#     static const std::vector<std::vector<double>> SOUTH_FACING_ROUNDABOUT = {{{15.737-rdb_ofs1, 11.690741+rdb_ofs2}}};
rdb_ofs1 = 0.107834
rdb_ofs2 = 0.05361
east_facing_roundabout = [
    [14.9777 - rdb_ofs2, 10.263 - rdb_ofs1, "Roundabout", "EAST"]
]
north_facing_roundabout = [
    [16.4 + rdb_ofs1, 9.52 - rdb_ofs2, "Roundabout", "SOUTH"]
]
west_facing_roundabout = [
    [17.164 + rdb_ofs2, 10.928 + rdb_ofs1, "Roundabout", "WEST"]
]
south_facing_roundabout = [
    [15.737 - rdb_ofs1, 11.690741 + rdb_ofs2, "Roundabout", "NORTH"]
]

cw_ofs2 = 0.025
cw_ofs1 = 0.028 + pole_size
east_facing_crosswalks = [
    [8.1675 - cw_ofs2, 0.7827 - cw_ofs1, "Crosswalk", "EAST"],
    [1.196 - cw_ofs2, 9.427 - cw_ofs1, "Crosswalk", "EAST"]
]
west_facing_crosswalks = [
    [1.76 + cw_ofs2, 10.16 + cw_ofs1, "Crosswalk", "WEST"],
    [9.521 + cw_ofs2, 4.157 + cw_ofs1, "Crosswalk", "WEST"]
]
south_facing_crosswalks = [
    [15.166 - cw_ofs1, 3.01 + cw_ofs2, "Crosswalk", "SOUTH"],
    [4.6255 - cw_ofs1, 7.9375 + cw_ofs2, "Crosswalk", "SOUTH"]
]
north_facing_crosswalks = [
    [17.253 + cw_ofs1, 2.313 - cw_ofs2, "Crosswalk", "NORTH"],
    [5.371 + cw_ofs1, 7.3775 - cw_ofs2, "Crosswalk", "NORTH"]
]

hw_entrance_east = [[6.3477, 11.56, "Highway Entrance", "EAST"]]
hw_entrance_west = [[14.35, 11.03, "Highway Entrance", "WEST"]]
hw_exit_east = [[13.63, 9.7, "Highway Exit", "EAST"]]
hw_exit_west = [[7.0845, 12.9, "Highway Exit", "WEST"]]

oneway_east = [[5.38, 0.123, "Oneway", "EAST"]]
oneway_west = [[14.76, 4.28, "Oneway", "WEST"]]
   
cars = [
    [9.4, 1.33, "Car", 0],
    [9.4, 0.64, "Car", math.pi],
    [10.94, 1.33, "Car", math.pi],
    [12.5, 0.63, "Car", math.pi],
    [15.65, 6.04, "Car", math.pi/2],
    [19.75, 10.52, "Car", 0.4874],
    [10.91, 10.7, "Car", -0.612],
    [9.97, 11.64, "Car", 2.622],
    [3.5, 9.77, "Car", -0.34],
    [4.865, 9.2, "Car", 2.05],
    [12.056, 4.04, "Car", 3.07],
    [3.947, 6.54, "Car", 0],
] 
            
# Collecting all data
data = []
data.extend(store_coordinates_with_context(north_facing_lane_centers, math.pi / 2))
data.extend(store_coordinates_with_context(south_facing_lane_centers, 3 * math.pi / 2))
data.extend(store_coordinates_with_context(east_facing_lane_centers, 0))
data.extend(store_coordinates_with_context(west_facing_lane_centers, math.pi))
data.extend(store_coordinates_with_context(south_facing_intersections, 3 * math.pi / 2))
data.extend(store_coordinates_with_context(north_facing_intersections, math.pi / 2))
data.extend(store_coordinates_with_context(west_facing_intersections, math.pi))
data.extend(store_coordinates_with_context(east_facing_intersections, 0))
data.extend(store_coordinates_with_context(south_facing_signs, 3 * math.pi / 2))
data.extend(store_coordinates_with_context(north_facing_signs, math.pi / 2))
data.extend(store_coordinates_with_context(west_facing_signs, math.pi))
data.extend(store_coordinates_with_context(east_facing_signs, 0))
data.extend(store_coordinates_with_context(south_facing_lights, 3 * math.pi / 2))
data.extend(store_coordinates_with_context(north_facing_lights, math.pi / 2))
data.extend(store_coordinates_with_context(west_facing_lights, math.pi))
data.extend(store_coordinates_with_context(east_facing_lights, 0))
data.extend(store_coordinates_with_context(parking_signs, 0))
data.extend(store_coordinates_with_context(east_facing_roundabout, 0))
data.extend(store_coordinates_with_context(north_facing_roundabout, math.pi / 2))
data.extend(store_coordinates_with_context(west_facing_roundabout, math.pi))
data.extend(store_coordinates_with_context(south_facing_roundabout, 3 * math.pi / 2))
data.extend(store_coordinates_with_context(east_facing_crosswalks, 0))
data.extend(store_coordinates_with_context(west_facing_crosswalks, math.pi))
data.extend(store_coordinates_with_context(south_facing_crosswalks, 3 * math.pi / 2))
data.extend(store_coordinates_with_context(north_facing_crosswalks, math.pi / 2))
data.extend(store_coordinates_with_context(hw_entrance_east, 0))
data.extend(store_coordinates_with_context(hw_entrance_west, math.pi))
data.extend(store_coordinates_with_context(hw_exit_east, 0))
data.extend(store_coordinates_with_context(hw_exit_west, math.pi))
data.extend(store_coordinates_with_context(oneway_east, 0))
data.extend(store_coordinates_with_context(oneway_west, math.pi))
for car in cars:
    data.append(car)
    
# data format
# <node id="2">
#   <data key="d0">4.17</data>
#   <data key="d1">7.24</data>
#   <data key="d2">7</data>
# </node>
# <node id="3">
#   <data key="d0">4.76</data>
#   <data key="d1">7.82</data>
#   <data key="d2">0</data>
# </node>
current_path = os.path.dirname(os.path.realpath(__file__))
with open(os.path.join(current_path + '/../../planning/scripts/config/paths.yaml'), 'r') as stream:
    data1 = yaml.safe_load(stream)
    destinations = data1['destinations']
G = nx.read_graphml(current_path + '/../../planning/scripts/maps/Competition_track_graph_modified_new.graphml')
for node_number in destinations:
    node = G.nodes[str(node_number)]
    data.append([node['x'], 13.786-node['y'], "Destination", 0])

# Create DataFrame and save it as a CSV
df = pd.DataFrame(data, columns=['X', 'Y', 'Type', 'Orientation'])
df.to_csv(os.path.join(current_path, 'assets', 'coordinates_with_context.csv'), index=False)
# df.to_csv('assets/coordinates_with_context.csv', index=False)