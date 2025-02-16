import networkx as nx
import numpy as np
import os
import yaml
import math
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

def plot_path_on_map(graph, path, map_image_path):
    # Load the map image
    img = mpimg.imread(map_image_path)

    # Create a plot with the map as background
    fig, ax = plt.subplots()
    ax.imshow(img, extent=[0, 20.696, 0, 13.786])  # Adjust these extents according to your map image's scaling

    # Plot the shortest path between consecutive nodes in the path
    for i in range(len(path) - 1):
        start_node = str(path[i])
        end_node = str(path[i + 1])
        sub_path = nx.shortest_path(graph, start_node, end_node, weight='length')

        # Extract the X and Y coordinates for each node in the sub_path
        x_coords = [graph.nodes[node]['x'] for node in sub_path]
        y_coords = [graph.nodes[node]['y'] for node in sub_path]
        y_coords = [13.786 - y for y in y_coords]  # Invert Y coordinates to match map scaling

        # Plot the edges between nodes
        ax.plot(x_coords, y_coords, 'o-', color='blue', markersize=5, label='Path' if i == 0 else "")

    # Plot and annotate each node with its sequence in the path
    for idx, node in enumerate(path):
        x, y = graph.nodes[str(node)]['x'], 13.786 - graph.nodes[str(node)]['y']
        ax.plot(x, y, 'ro')  # Red dot for each node
        ax.annotate(f"{idx + 1}", (x, y), textcoords="offset points", xytext=(0, 5), ha='center', fontsize=8, color='white')  # Sequence number

    # Annotate the start and end points
    start_coords = graph.nodes[str(path[0])]
    start_x, start_y = start_coords['x'], 13.786 - start_coords['y']
    ax.annotate("Start", (start_x, start_y), textcoords="offset points", xytext=(0, 10), ha='center', color='green')

    end_coords = graph.nodes[str(path[-1])]
    end_x, end_y = end_coords['x'], 13.786 - end_coords['y']
    ax.annotate("End", (end_x, end_y), textcoords="offset points", xytext=(0, 10), ha='center', color='red')

    # Show the plot
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Optimal Path on the Map (Following Graph Edges)')
    plt.legend()
    plt.show()


class GlobalPlanner:
    def __init__(self):
        self.hw_safety_offset = 0.05
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_modified_new.graphml')
        self.pos = {}
        self.attribute = {}
        for node, data in self.G.nodes(data=True):
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            self.pos[node] = (x, 13.786 - y)
            self.attribute[node] = data.get('new_attribute', 0)
        
        # Compute edge weights based on Euclidean distance
        for u, v in self.G.edges():
            if u in self.pos and v in self.pos:
                x1, y1 = self.pos[u]
                x2, y2 = self.pos[v]
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                if self.attribute[u] in [4, 5] or self.attribute[v] in [4, 5]:
                    # In highway, distance is scaled
                    distance *= 0.5
                self.G[u][v]['weight'] = distance
            else:
                self.G[u][v]['weight'] = float('inf')
        
        current_dir = os.path.dirname(os.path.realpath(__file__))
        destination_path = os.path.join(current_dir, 'config/destinations_mod_condensed.yaml')
        with open(destination_path, 'r') as f:
            self.base_destinations = yaml.safe_load(f)
        self.base_destinations = [str(d) for d in self.base_destinations]
        print(f"Base destinations: {self.base_destinations}")
        
        self.destinations = self.base_destinations.copy()

    def plan_path(self, start, end):
        if not isinstance(start, str):
            start = str(start)
        if not isinstance(end, str):
            end = str(end)
        path = nx.dijkstra_path(self.G, source=start, target=end)
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        wp_x = []
        wp_y = []
        wp_attributes = []
        for node in path:
            attribute = self.attribute.get(node, 0)
            wp_attributes.append(attribute)
            if node in self.pos:
                x, y = self.pos[node]
                wp_x.append(x)
                wp_y.append(y)
        return np.array([wp_x, wp_y]), path_edges, wp_attributes

    def find_optimal_sequence(self, start_node, max_distance=80.0):
        """
        Attempts to find a route (sequence of nodes) that visits all destinations (plus the start)
        without exceeding max_distance. This method uses recursive backtracking to explore all 
        orders of visiting the destinations while pruning any path that would exceed the budget.
        
        If a complete solution (visiting all nodes) is found under the limit, it is returned.
        Otherwise, we append one of the missing destinations (the one that minimizes extra distance)
        to the best partial solution.
        """
        # Prepare the list of nodes: start_node plus all base destinations (if start_node is among them, remove it)
        self.destinations = self.base_destinations.copy()
        if start_node in self.destinations:
            print(f"Removing start node {start_node} from destinations")
            self.destinations.remove(start_node)
        full_nodes = [start_node] + self.destinations

        # Build a distance matrix for the nodes in full_nodes using Dijkstra.
        self.distance_matrix = {}
        for u in full_nodes:
            self.distance_matrix[u] = {}
            for v in full_nodes:
                if u == v:
                    self.distance_matrix[u][v] = 0.0
                else:
                    try:
                        length = nx.dijkstra_path_length(self.G, u, v, weight='weight')
                        self.distance_matrix[u][v] = length
                    except nx.NetworkXNoPath:
                        self.distance_matrix[u][v] = float('inf')

        # Global variables to track the best complete (all destinations visited)
        # and the best partial solution.
        best_complete_path = None
        best_complete_distance = float('inf')
        best_partial_path = None
        best_partial_num_visited = 0
        best_partial_distance = float('inf')

        def backtrack(current, visited, current_distance, path):
            nonlocal best_complete_path, best_complete_distance, best_partial_path, best_partial_num_visited, best_partial_distance

            # Update best partial solution if this path visits more nodes
            if (len(path) > best_partial_num_visited) or (len(path) == best_partial_num_visited and current_distance < best_partial_distance):
                best_partial_num_visited = len(path)
                best_partial_distance = current_distance
                best_partial_path = path.copy()

            # If all nodes have been visited, record complete solution.
            if len(visited) == len(full_nodes):
                if current_distance < best_complete_distance:
                    best_complete_path = path.copy()
                    best_complete_distance = current_distance
                return

            # Try each candidate node not yet visited.
            for candidate in full_nodes:
                if candidate not in visited:
                    d = self.distance_matrix[current][candidate]
                    if current_distance + d <= max_distance:
                        visited.add(candidate)
                        path.append(candidate)
                        backtrack(candidate, visited, current_distance + d, path)
                        path.pop()
                        visited.remove(candidate)

        visited = set([start_node])
        backtrack(start_node, visited, 0.0, [start_node])

        # If a complete solution was found, return it.
        if best_complete_path is not None:
            print(f"Found complete path with distance {best_complete_distance:.2f}m")
            return best_complete_path, best_complete_distance
        else:
            # Otherwise, determine which destinations are missing.
            missing = set(self.destinations) - set(best_partial_path)
            if missing:
                # Try appending the missing destination that increases the route distance minimally.
                last_node = best_partial_path[-1]
                best_extra = float('inf')
                best_candidate = None
                for candidate in missing:
                    d_extra = self.distance_matrix[last_node].get(candidate, float('inf'))
                    if d_extra < best_extra:
                        best_extra = d_extra
                        best_candidate = candidate
                if best_candidate is not None and best_extra < float('inf'):
                    best_partial_path.append(best_candidate)
                    best_partial_distance += best_extra
            print(f"Added missing destination {best_candidate} with extra distance {best_extra:.2f}m")
            return best_partial_path, best_partial_distance


if __name__ == "__main__":
    planner = GlobalPlanner()
    
    current_dir = os.path.dirname(os.path.realpath(__file__))
    starting_points_path = os.path.join(current_dir, 'config/starting_points.yaml')
    with open(starting_points_path, 'r') as f:
        starting_points = yaml.safe_load(f)
    
    runs = {}
    
    max_dist = 75.0  # maximum allowed distance in meters
    
    for start in starting_points:
        start_str = str(start)
        optimal_path, total_dist = planner.find_optimal_sequence(start_str, max_dist)
        # Convert nodes to integers if needed (and remove the start node from the final output)
        optimal_path_ints = [int(node) for node in optimal_path]
        if optimal_path_ints and optimal_path_ints[0] == int(start):
            optimal_path_ints.pop(0)
        runs[f'run{start}'] = optimal_path_ints
        print(f"run{start}: distance={total_dist:.2f}m, num_destinations={len(optimal_path)-1}")
    
    runs_path = os.path.join(current_dir, 'config/runs.yaml')
    with open(runs_path, 'w') as f:
        yaml.dump(runs, f, default_flow_style=False)
