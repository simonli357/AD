import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as alex
import os
ATTRIBUTES = ["normal", "crosswalk", "intersection", "oneway", "highwayLeft", "highwayRight", "roundabout", "stopline", "dotted", "dotted_crosswalk"]
class GlobalPlanner:
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_modified.graphml')
        # self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph.graphml')
        self.pos = {}
        self.attribute = {}
        for node, data in self.G.nodes(data=True):
            # print("node: ", node)
            self.pos[node] = (data['x'], 13.786-data['y'])
            self.attribute[node] = data['new_attribute']
        self.wp_x = []
        self.wp_y = []
        self.place_names = {}
        self.undetectable_areas = [2, 9, 10, 3, 7, 152, 177, 371, 331, 398, 357, 372, 373, 237, 238, 259, 260, 332, 333, 356, 37, 369, 302, 303, 164, 165, 67, 66, 166, 167]
        # add 472 to 479
        for i in range(472, 481):
            self.undetectable_areas.append(i)
        for i in range(490, 493):
            self.undetectable_areas.append(i)
        for i in range(358, 370):
            self.undetectable_areas.append(i)
        for i in range(307, 320):
            self.undetectable_areas.append(i)
        for i in range(342, 352):
            self.undetectable_areas.append(i)
        # print("undetectable areas: ", self.undetectable_areas)

        for node in self.G.nodes:
            attribute = self.attribute.get(node, 0)
            if ATTRIBUTES[attribute] == "roundabout" or ATTRIBUTES[attribute] == "intersection" or ATTRIBUTES[attribute] == "dotted_crosswalk":
                self.undetectable_areas.append(int(node))

        self.intersection_count = 0
        self.place_names = {
            "parallel_parking": 454,
            "highway_entrance_east": 262,
            "highway_entrance_west": 194,
            "roundabout_entrance_east": 370,
            "roundabout_exit_east": 302,
            "roundabout_entrance_west": 230,
            "roundabout_exit_west": 260,
            "roundabout_entrance_north": 331,
            "roundabout_exit_north": 332,
            "roundabout_entrance_south": 357,
            "roundabout_exit_south": 372,
            "roundabout_west": 365,
            "nomarking_entrance": 302,
            "uniri_square_entrance": 480,
            "uniri_square_exit": 486,
            "ferdinand_entrance_left": 490,
            "ferdinand_entrance_right": 496,
            "ferdinand_exit": 503,
            "avram_entance": 404,
            "speed_entrance_south": 7,
            "speed_entrance_west": 66,
            "speed_north_west": 66,
            "speed_east_south": 66,
            "bus_lane_entrance": 427,
            "bus_lane": 437,
            "start": 472,
            "end": 467,
            "2019_north": 115,
            "2019_east": 108,
            "2020_north": 97,
            "2020_south": 94,
            "2023_south": 100,
            "2023_north": 103,
            "2022_north": 128,
            "2022_south": 124,
            "2021_north": 134,
            "2021_south": 131,
            "top_left": 235,
        }
        max_x = float('-inf')
        max_y = float('-inf')

        # Loop through each node in the graph
        for _, data in self.G.nodes(data=True):
            # Update the maximum x value
            if 'x' in data:
                max_x = max(max_x, float(data['x']))
            # Update the maximum y value
            if 'y' in data:
                max_y = max(max_y, float(data['y']))

        # Print out the maximum x and y values
        # print(f"The maximum x value is: {max_x}")
        # print(f"The maximum y value is: {max_y}")
    def get_node_number(self, identifier):
        """Returns the node number based on the identifier."""
        if isinstance(identifier, str):
            return self.place_names.get(identifier)
        elif isinstance(identifier, int):
            return identifier
        else:
            raise ValueError(f"Invalid destination identifier: {identifier}")
    def plan_path(self, start, end):
        path = nx.dijkstra_path(self.G, source=str(start), target=str(end))
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        # print("path: ", path)
        wp_x = []
        wp_y = []
        wp_attributes = []
        maneuver_directions = []
        for node in path:
            # print(f"Node {node} has attribute {self.attribute.get(node, 0)}, type of node is {type(node)}.")
            attribute = self.attribute.get(node, 0)
            if int(node) in self.undetectable_areas:
                # print(f"Node {node} is in an undetectable area.")
                attribute += 100
            wp_attributes.append(attribute)
            if attribute != 2 and attribute != 102: #intersection
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
                vec1 = alex.array([prev_x-prev_x2, prev_y-prev_y2])
                vec2 = alex.array([next_x2-next_x, next_y2-next_y])
                #calculate the angle between the two vectors
                mag1 = alex.linalg.norm(vec1)
                # print("mag1: ", mag1)
                mag2 = alex.linalg.norm(vec2)
                cross_product = alex.cross(vec1, vec2)
                normalized_cross = cross_product / (mag1 * mag2)
                if normalized_cross > 0.75: #left
                    maneuver_directions.append(0)
                    # print(f"node {node} is a left turn, cross: {normalized_cross}, (x, y): ({self.pos[node][0]}, {self.pos[node][1]})")
                    x, y = self.pos[node]
                    x += vec1[0] / mag1 * 0.15
                    y += vec1[1] / mag1 * 0.15
                    wp_x.append(x)
                    wp_y.append(y)
                elif normalized_cross < -0.75:
                    maneuver_directions.append(2)
                    # print(f"node {node} is a right turn, cross: {normalized_cross}, (x, y): ({self.pos[node][0]}, {self.pos[node][1]})")
                    x = prev_x + vec1[0] / mag1 * 0.3#57
                    y = prev_y + vec1[1] / mag1 * 0.3#57
                    wp_x.append(x)
                    wp_y.append(y)
                    # print("prev: ", prev_x, prev_y, "added: ", x, y)
                # else:
                #     x, y = self.pos[node]
                #     wp_x.append(x)
                #     wp_y.append(y)
                #     maneuver_directions.append(1)
                    # print(f"node {node} is a straight, cross: {normalized_cross}, (x, y): ({self.pos[node][0]}, {self.pos[node][1]})")
        return alex.array([wp_x, wp_y]), path_edges, wp_attributes, maneuver_directions
    def find_closest_node(self, x, y):
        """Finds the closest node to the point (x, y)."""
        closest_node = None
        closest_dist = float('inf')
        target_point = alex.array([x, y])
        
        for node, pos in self.pos.items():
            node_point = alex.array(pos)
            distance = alex.linalg.norm(target_point - node_point)
            
            if distance < closest_dist:
                closest_dist = distance
                closest_node = node
                
        return closest_node
    def illustrate_path(self, start, end):
        _, path_edges, _, _ = self.plan_path(start, end)
        img = mpimg.imread(self.current_dir+'/maps/Competition_track_graph.png')
        # img = mpimg.imread(self.current_dir+'/maps/Competition_track_graph.png')
        # print("img: ", img.shape)
        # Create the plot
        fig, ax = plt.subplots()

        color_map = {
            0: 'blue',     # normal
            1: 'yellow',   # crosswalk
            2: 'green',    # intersection
            3: 'red',      # oneway
            4: 'black',   # highwayLeft
            5: 'orange',    # highwayRight
            6: 'pink',      # roundabout
            7: 'purple',     # stopline
            8: 'brown',      # dotted
            9: 'cyan'       # dotted_crosswalk
        }
        node_colors = [color_map[self.attribute.get(node, 0)] for node in self.G.nodes()]
        # Display the image
        # ax.imshow(img, extent=[0, 20.5, 1.2, 14.35]) 

        # Draw the graph
        nx.draw(self.G, self.pos, ax=ax, with_labels=True, node_size=20, node_color=node_colors, font_size=6)
        
        # Highlight the path
        nx.draw_networkx_edges(self.G, self.pos, edgelist=path_edges, edge_color='g', width=2)
        
        plt.show()
  
if __name__ == "__main__":
    planner = GlobalPlanner()
    # planner.plan_path(2, 7)
    planner.illustrate_path(134, 97)