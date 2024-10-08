import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import os

# Function to retrieve coordinates from the graph
def get_node_coordinates(graph, destinations):
    coordinates = []
    for node in destinations:
        if str(node) in graph.nodes:
            x, y = graph.nodes[str(node)]['x'], graph.nodes[str(node)]['y']
            coordinates.append((x, y))
        else:
            raise ValueError(f"Node {node} not found in graph.")
    return np.array(coordinates)

def create_graph_distance_matrix(graph, destinations):
    num_nodes = len(destinations)
    distance_matrix = np.zeros((num_nodes, num_nodes))
    
    for i in range(num_nodes):
        for j in range(i, num_nodes):
            if i == j:
                distance = 0
            else:
                distance = nx.shortest_path_length(graph, str(destinations[i]), str(destinations[j]), weight='length')
            distance_matrix[i][j] = distance
            distance_matrix[j][i] = distance  # Distance is symmetric
    
    return distance_matrix

# OR-Tools TSP solver
def solve_tsp_with_ortools(distance_matrix):
    num_locations = len(distance_matrix)
    
    # Create the routing index manager
    manager = pywrapcp.RoutingIndexManager(num_locations, 1, 0)
    
    # Create the routing model
    routing = pywrapcp.RoutingModel(manager)
    
    # Create and register a transit callback
    def distance_callback(from_index, to_index):
        from_node = manager.IndexToNode(from_index)
        to_node = manager.IndexToNode(to_index)
        return int(distance_matrix[from_node][to_node])
    
    transit_callback_index = routing.RegisterTransitCallback(distance_callback)
    
    # Define cost of each arc
    routing.SetArcCostEvaluatorOfAllVehicles(transit_callback_index)
    
    # Set search parameters
    search_parameters = pywrapcp.DefaultRoutingSearchParameters()
    search_parameters.first_solution_strategy = (
        routing_enums_pb2.FirstSolutionStrategy.PATH_CHEAPEST_ARC)
    
    # Solve the problem
    solution = routing.SolveWithParameters(search_parameters)
    
    # Return the solution path
    if solution:
        index = routing.Start(0)
        path = []
        while not routing.IsEnd(index):
            path.append(manager.IndexToNode(index))
            index = solution.Value(routing.NextVar(index))
        return path
    else:
        return None

def plot_path_on_map(graph, coordinates, destinations, path, map_image_path):
    # Load the map image
    img = mpimg.imread(map_image_path)
    
    # Create a plot with the map as background
    fig, ax = plt.subplots()
    ax.imshow(img, extent=[0, 20.696, 0, 13.786])  # Adjust these extents according to your map image's scaling
    
    # Plot the shortest path between consecutive nodes in the path
    for i in range(len(path) - 1):
        start_node = str(destinations[path[i]])
        end_node = str(destinations[path[i + 1]])
        sub_path = nx.shortest_path(graph, start_node, end_node, weight='length')
        
        # Extract the X and Y coordinates for each node in the sub_path
        x_coords = [graph.nodes[node]['x'] for node in sub_path]
        y_coords = [graph.nodes[node]['y'] for node in sub_path]
        y_coords = [13.786-y for y in y_coords]
        
        # Plot the edges between nodes
        ax.plot(x_coords, y_coords, 'o-', color='blue', markersize=5, label='Path' if i == 0 else "")
    
    # Plot and annotate each node with its sequence in the path
    for idx, node_idx in enumerate(path):
        node = str(destinations[node_idx])
        x, y = graph.nodes[node]['x'], 13.786-graph.nodes[node]['y']
        ax.plot(x, y, 'ro')  # Red dot for each node
        ax.annotate(f"{idx + 1}", (x, y), textcoords="offset points", xytext=(0,5), ha='center', fontsize=8, color='white')  # Sequence number

    # Annotate the start and end points
    start_coords = coordinates[path[0]]
    # start_coords = (start_coords[0], 13.786-start_coords[1])
    end_coords = coordinates[path[-1]]
    # end_coords = (end_coords[0], 13.786-end_coords[1])
    ax.annotate("Start", (start_coords[0], start_coords[1]), textcoords="offset points", xytext=(0,10), ha='center', color='green')
    ax.annotate("End", (end_coords[0], end_coords[1]), textcoords="offset points", xytext=(0,10), ha='center', color='red')
    
    # Show the plot
    plt.xlabel('X')
    plt.ylabel('Y')
    plt.title('Optimal Path on the Map (Following Graph Edges)')
    plt.legend()
    plt.show()

# Main function
def main():
    # Define your destination nodes
    destinations = [386, 343, 362, 368, 317, 318, 404, 399, 425, 420, 437, 82, 80, 93, 121, 116, 127, 75, 71, 
                    185, 27, 25, 31, 49, 301, 8, 289, 199, 42, 225, 228, 239, 261, 257, 56]
    
    # Load the graph (adjust the file path accordingly)
    graph_file = '/maps/Competition_track_graph_modified_new.graphml'
    current_dir = os.path.dirname(os.path.realpath(__file__))
    graph = nx.read_graphml(current_dir + graph_file)
    
    # Get node coordinates for destinations
    coordinates = get_node_coordinates(graph, destinations)
    #invert y coordinates
    coordinates = [(x, 13.786-y) for x, y in coordinates]
    
    distance_matrix = create_graph_distance_matrix(graph, destinations)
    
    # Solve the TSP problem using OR-Tools
    best_path = solve_tsp_with_ortools(distance_matrix)
    
    if best_path:
        print("Best Path (order of node indices):", [destinations[i] for i in best_path])
        
        plot_path_on_map(graph, coordinates, destinations, best_path, current_dir + '/maps/Track.png')
    else:
        print("No solution found!")

if __name__ == "__main__":
    main()
