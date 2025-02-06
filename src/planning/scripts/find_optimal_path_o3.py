import networkx as nx
import numpy as np
import os
import yaml
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
        self.hw_safety_offset = 0.05  # 0.1
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_modified_new.graphml')
        self.pos = {}
        self.attribute = {}
        for node, data in self.G.nodes(data=True):
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            # Adjust y so that the map is oriented correctly.
            self.pos[node] = (x, 13.786 - y)  # 13.786 is the height of the map
            self.attribute[node] = data.get('new_attribute', 0)
        
        # Optional: Add edge weights if they are not already defined.
        # Here we use Euclidean distance between node positions.
        self._add_edge_weights()
                
        current_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(current_path + '/../../planning/scripts/config/paths.yaml'), 'r') as stream:
            data1 = yaml.safe_load(stream)
            self.destinations = data1['destinations']
            
    def _add_edge_weights(self):
        """Adds a 'weight' attribute to each edge equal to the Euclidean distance between nodes."""
        for u, v, data in self.G.edges(data=True):
            # Only add weight if it isn’t already present.
            if 'weight' not in data:
                if u in self.pos and v in self.pos:
                    x1, y1 = self.pos[u]
                    x2, y2 = self.pos[v]
                    data['weight'] = np.hypot(x2 - x1, y2 - y1)
                else:
                    data['weight'] = 1  # fallback

    def plan_path(self, start, end):
        """Plans a path between two nodes using Dijkstra's algorithm."""
        # Make sure nodes are strings (as in your graph)
        start = str(start)
        end = str(end)
        path = nx.dijkstra_path(self.G, source=start, target=end, weight='weight')
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

    def plan_route_with_budget(self, start, destinations, budget=150.0):
        """
        Heuristic: Starting at 'start' (node 112) and with a budget of 150 meters,
        greedily select the next destination (from the list) that is reachable
        within the remaining budget.
        
        Note: This does not guarantee an optimal solution but is a simple heuristic.
        """
        start = str(start)
        # Convert all destination nodes to strings and remove the start if present.
        unvisited = {str(d) for d in destinations if str(d) != start}
        route = [start]
        total_distance = 0.0
        current = start
        
        while unvisited:
            best_candidate = None
            best_distance = float('inf')
            # Look for the closest unvisited destination that we can still reach
            for candidate in unvisited:
                try:
                    d = nx.dijkstra_path_length(self.G, source=current, target=candidate, weight='weight')
                except nx.NetworkXNoPath:
                    continue  # Skip if no path exists.
                if d < best_distance and (total_distance + d) <= budget:
                    best_distance = d
                    best_candidate = candidate
            if best_candidate is None:
                # No further destination can be reached within the budget.
                break
            # Append the best candidate to our route.
            route.append(best_candidate)
            total_distance += best_distance
            current = best_candidate
            unvisited.remove(best_candidate)
        return route, total_distance

if __name__ == "__main__":
    planner = GlobalPlanner()
    # Our list of destinations
    destinations = [112, 386, 343, 362, 368, 317, 318, 404, 399, 425, 420, 437, 82, 80, 
                    93, 121, 116, 127, 75, 71, 185, 27, 25, 31, 29, 301, 8, 289, 199, 42, 
                    225, 228, 239, 261, 257, 56]
    
    # Start at node 112 and plan a route with a 150 meter travel budget.
    route, distance = planner.plan_route_with_budget(start=112, destinations=destinations, budget=150.0)
    print("Planned route:", route)
    print("Number of destinations:", len(route))
    print("Total travel distance:", distance)
    
    # Plot the optimal path on the map
    current_dir = os.path.dirname(os.path.realpath(__file__))
    plot_path_on_map(planner.G, route, current_dir + '/maps/Competition_track_graph_new.png')
