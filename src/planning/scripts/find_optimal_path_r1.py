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
                self.G[u][v]['weight'] = distance
            else:
                self.G[u][v]['weight'] = float('inf')
        
        # Define destinations from the problem statement, converting to strings
        self.destinations = ['112', '386', '343', '362', '368', '317', '318', '404', '399', 
                             '425', '420', '437', '82', '80', '93', '121', '116', '127', '75', 
                             '71', '185', '27', '25', '31', '29', '301', '8', '289', '199', 
                             '42', '225', '228', '239', '261', '257', '56']
        
        # Precompute distance matrix between all pairs of destinations
        self.distance_matrix = {}
        for start in self.destinations:
            self.distance_matrix[start] = {}
            for end in self.destinations:
                if start == end:
                    self.distance_matrix[start][end] = 0.0
                else:
                    try:
                        length = nx.dijkstra_path_length(self.G, start, end, weight='weight')
                        self.distance_matrix[start][end] = length
                    except nx.NetworkXNoPath:
                        self.distance_matrix[start][end] = float('inf')
    
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

    def find_optimal_sequence(self):
        start_node = '112'
        if start_node not in self.destinations:
            return [], 0.0
        
        remaining_budget = 150.0
        current_node = start_node
        visited = set([current_node])
        path = [current_node]
        total_distance = 0.0

        # Create a list of destination candidates excluding the start node
        candidates = [d for d in self.destinations if d != start_node]
        
        while True:
            best_next = None
            best_score = float('inf')  # Lower score is better (distance + minimal next hop)
            
            for candidate in list(candidates):  # Iterate over a copy to avoid modification issues
                if candidate in visited:
                    continue
                
                # Get distance from current node to candidate
                dist_current_to_candidate = self.distance_matrix[current_node].get(candidate, float('inf'))
                if total_distance + dist_current_to_candidate > remaining_budget:
                    continue  # Skip if adding this candidate exceeds the budget
                
                # Calculate the minimal distance from candidate to any other unvisited candidate
                min_next_dist = float('inf')
                for next_candidate in candidates:
                    if next_candidate == candidate or next_candidate in visited:
                        continue
                    dist = self.distance_matrix[candidate].get(next_candidate, float('inf'))
                    if dist < min_next_dist:
                        min_next_dist = dist
                
                # Score is the current distance plus the minimal next hop
                score = dist_current_to_candidate + (min_next_dist if min_next_dist != float('inf') else 0)
                
                # Prefer lower score (allows more remaining budget)
                if score < best_score:
                    best_next = candidate
                    best_score = score
            
            if best_next is not None:
                # Add the best next candidate to the path
                dist = self.distance_matrix[current_node][best_next]
                path.append(best_next)
                visited.add(best_next)
                total_distance += dist
                current_node = best_next
                candidates.remove(best_next)
                
                # Check if remaining budget allows for at least one more minimal hop
                remaining = remaining_budget - total_distance
                if remaining <= 0:
                    break
            else:
                break  # No more candidates can be added
        
        return path, total_distance

if __name__ == "__main__":
    planner = GlobalPlanner()
    optimal_path, total_dist = planner.find_optimal_sequence()
    print(f"Optimal sequence of destinations: {optimal_path}")
    print(f"Total distance traveled: {total_dist} meters")
    print(f"Number of destinations visited: {len(optimal_path)}")
    
    # Plot the optimal path on the map
    plot_path_on_map(planner.G, optimal_path, planner.current_dir + '/maps/Competition_track_graph_new.png')