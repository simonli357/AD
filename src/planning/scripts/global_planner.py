import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import yaml

ATTRIBUTES = ["normal", "crosswalk", "intersection", "oneway", "highwayLeft", "highwayRight", "roundabout", "stopline", "dotted", "dotted_crosswalk"]

class GlobalPlanner:
    def __init__(self):
        self.hw_safety_offset = 0.05#0.1
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_modified_new.graphml')
        self.pos = {}
        self.attribute = {}
        for node, data in self.G.nodes(data=True):
            x = data.get('x', 0.0)  # Default value 0.0 if 'x' is missing
            if 502 <= int(node) <= 521:
                y = data.get('y', 0.0) + self.hw_safety_offset
            elif 483 <= int(node) <= 502:
                y = data.get('y', 0.0) - self.hw_safety_offset
            else:
                y = data.get('y', 0.0)  # Default value 0.0 if 'y' is missing
            self.pos[node] = (x, 13.786 - y)
            self.attribute[node] = data.get('new_attribute', 0)
        
        self.wp_x = []
        self.wp_y = []
        self.place_names = {}
        self.undetectable_areas = [398, 399, 403, 404, 405, 400, 366, 367, 368, 369, 342, 343, 396, 397, 318, 317, 316]
        
        for i in range(263, 271):
            self.undetectable_areas.append(i)
        for i in range(292, 296):
            self.undetectable_areas.append(i)
        # for i in range(358, 370):
        #     self.undetectable_areas.append(i)
        for i in range(373, 386):
            self.undetectable_areas.append(i)
        for i in range(353, 363):
            self.undetectable_areas.append(i)
        
        for node in self.G.nodes:
            attribute = self.attribute.get(node, 0)
            if ATTRIBUTES[attribute] == "roundabout" or ATTRIBUTES[attribute] == "intersection" or ATTRIBUTES[attribute] == "dotted_crosswalk":
                self.undetectable_areas.append(int(node))
        
        self.intersection_count = 0
        self.place_names = {
            "highway_entrance_east": 401,
            "highway_entrance_west": 423,
            "roundabout_entrance_east": 367,
            "roundabout_exit_east": 368,
            "roundabout_entrance_west": 405,
            "roundabout_exit_west": 398,
            "roundabout_entrance_north": 397,
            "roundabout_exit_north": 342,
            "roundabout_entrance_south": 317,
            "roundabout_exit_south": 318,
            "roundabout_west": 337,
            "nomarking_entrance": 369,
            "speed_north_west": 157,
            "speed_east_south": 171,
            "bus_lane_entrance": 271,
            "bus_lane": 277,
            "start": 263,
            "end": 241
        }
        max_x = float('-inf')
        max_y = float('-inf')

        for _, data in self.G.nodes(data=True):
            if 'x' in data:
                max_x = max(max_x, float(data['x']))
            if 'y' in data:
                max_y = max(max_y, float(data['y']))
                
        current_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(current_path + '/../../planning/scripts/config/paths.yaml'), 'r') as stream:
            data1 = yaml.safe_load(stream)
            destinations = data1['destinations']
            
        return

    def get_node_number(self, identifier):
        if isinstance(identifier, str):
            if self.place_names.get(identifier) is not None:
                return self.place_names[identifier]
            else:
                return int(identifier)
        elif isinstance(identifier, int):
            return identifier
        else:
            raise ValueError(f"Invalid destination identifier: {identifier}")

    def get_node_coordinates(self, node):
        return np.array(self.pos[node])
    
    def get_distance(self, start, end):
        return nx.dijkstra_path_length(self.G, source=start, target=end, weight='weight')
    
    def get_total_distance(self, sequence):
        total_distance = 0.0
        for i in range(len(sequence) - 1):
            start = sequence[i]
            end = sequence[i + 1]
            total_distance += self.get_distance(start, end)
        return total_distance
    
    def plan_path(self, start, end):
        if not isinstance(start, str):
            start = str(start)
        if not isinstance(end, str):
            end = str(end)
        print("start: ", start, "end: ", end)
        path = nx.dijkstra_path(self.G, source=start, target=end)
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        print("path: ", path)
        wp_x = []
        wp_y = []
        wp_attributes = []
        for node in path:
            attribute = self.attribute.get(node, 0)
            if int(node) in self.undetectable_areas:
                attribute += 100
            wp_attributes.append(attribute)
            if attribute != 2 and attribute != 102:  # intersection
                if node in self.pos:
                    x, y = self.pos[node]
                    wp_x.append(x)
                    wp_y.append(y)
            else:
                self.intersection_count += 1
                # print("intersection node: ", node, "count: ", self.intersection_count)
                #get previous node
                prev_node2 = path[path.index(node)-2]
                prev_node = path[path.index(node)-1]
                #get next node
                next_node = path[path.index(node)+1]
                try:
                    next_node2 = path[path.index(node)+2]
                except:
                    print("end of path at node: ", node)
                    continue
                #calculate the vector from prev to current
                prev_x2, prev_y2 = self.pos[prev_node2]
                prev_x, prev_y = self.pos[prev_node]
                next_x, next_y = self.pos[next_node]
                next_x2, next_y2 = self.pos[next_node2]
                vec1 = np.array([prev_x-prev_x2, prev_y-prev_y2])
                vec2 = np.array([next_x2-next_x, next_y2-next_y])
                #calculate the angle between the two vectors
                mag1 = np.linalg.norm(vec1)
                # print("mag1: ", mag1)
                mag2 = np.linalg.norm(vec2)
                cross_product = np.cross(vec1, vec2)
                normalized_cross = cross_product / (mag1 * mag2)
                if normalized_cross > 0.75: #left
                    # print(f"node {node} is a left turn, cross: {normalized_cross}, (x, y): ({self.pos[node][0]}, {self.pos[node][1]})")
                    x, y = self.pos[node]
                    x += vec1[0] / mag1 * 0.2 #15
                    y += vec1[1] / mag1 * 0.2 #15
                    # adjust with vec2
                    x += vec2[0] / mag2 * 0.43 #15
                    y += vec2[1] / mag2 * 0.43 #15
                    wp_x.append(x)
                    wp_y.append(y)
                elif normalized_cross < -0.75:
                    # print(f"node {node} is a right turn, cross: {normalized_cross}, (x, y): ({self.pos[node][0]}, {self.pos[node][1]})")
                    # x = prev_x + vec1[0] / mag1 * 0.0015#0.001
                    # y = prev_y + vec1[1] / mag1 * 0.0015#0.001
                    x = prev_x + vec1[0] / mag1 * 0.4#0.001
                    y = prev_y + vec1[1] / mag1 * 0.4#0.001
                    # adjust with vec2
                    # x += vec2[0] / mag2 * 0.0005 #15
                    # y += vec2[1] / mag2 * 0.0005 #15
                    # x = prev_x + vec2[0] / mag2 * 0.001#25#57
                    # y = prev_y + vec2[1] / mag2 * 0.001#25#57
                    wp_x.append(x)
                    wp_y.append(y)
                else:
                    if int(node) == 80:
                        print("node 80 straight")
                        exit()
                    pass
        return np.array([wp_x, wp_y]), path_edges, wp_attributes

    def find_closest_node(self, x, y):
        closest_node = None
        closest_dist = float('inf')
        target_point = np.array([x, y])
        for node, pos in self.pos.items():
            node_point = np.array(pos)
            distance = np.linalg.norm(target_point - node_point)
            if distance < closest_dist:
                closest_dist = distance
                closest_node = node
        return closest_node

    def illustrate_path(self, start, end):
        _, path_edges, _ = self.plan_path(start, end)
        img = mpimg.imread(self.current_dir + '/maps/Track.png')
        fig, ax = plt.subplots()
        color_map = {
            0: 'blue',
            1: 'yellow',
            2: 'green',
            3: 'red',
            4: 'black',
            5: 'orange',
            6: 'pink',
            7: 'purple',
            8: 'brown',
            9: 'cyan'
        }
        node_colors = [color_map[self.attribute.get(node, 0)] for node in self.G.nodes()]
        nx.draw(self.G, self.pos, ax=ax, with_labels=True, node_size=20, node_color=node_colors, font_size=6)
        nx.draw_networkx_edges(self.G, self.pos, edgelist=path_edges, edge_color='g', width=2)
        plt.show()

if __name__ == "__main__":
    planner = GlobalPlanner()
    # planner.plan_path(18, 15)
    planner.illustrate_path(399, 427)
