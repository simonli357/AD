import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import os
from networkx.algorithms.matching import max_weight_matching

def get_node_coordinates(graph, destinations):
    coordinates = []
    for node in destinations:
        if str(node) in graph.nodes:
            x, y = graph.nodes[str(node)]['x'], graph.nodes[str(node)]['y']
            coordinates.append((x, y))
        else:
            raise ValueError(f"Node {node} not found in graph.")
    return np.array(coordinates)

def find_closest_node(graph, x, y):
    closest_node = None
    min_distance = float('inf')
    
    # Iterate through all nodes in the graph to find the closest node
    for node, data in graph.nodes(data=True):
        node_x = data.get('x')
        node_y = data.get('y')
        distance = np.sqrt((x - node_x)**2 + (y - node_y)**2)  # Euclidean distance
        
        if distance < min_distance:
            min_distance = distance
            closest_node = node
    
    return closest_node

def create_graph_hop_matrix(graph, destinations):
    num_nodes = len(destinations)
    hop_matrix = np.full((num_nodes, num_nodes), float('inf'))  # Initialize with infinity (no path)

    for i in range(num_nodes):
        for j in range(num_nodes):
            if i == j:
                hop_matrix[i][j] = 0  # No hops needed for the same node
            else:
                try:
                    # Use shortest path to find the number of hops between i and j considering uni-directionality
                    hops = nx.shortest_path_length(graph, str(destinations[i]), str(destinations[j]))
                    hop_matrix[i][j] = hops
                except nx.NetworkXNoPath:
                    # If no path exists, it remains float('inf') in the matrix
                    pass
    
    return hop_matrix

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

def solve_christofides_tsp(distance_matrix):
    num_nodes = len(distance_matrix)
    
    # Create a complete weighted graph from the distance matrix
    G = nx.Graph()
    for i in range(num_nodes):
        for j in range(i + 1, num_nodes):
            G.add_edge(i, j, weight=distance_matrix[i][j])

    # Step 1: Find the Minimum Spanning Tree (MST)
    mst = nx.minimum_spanning_tree(G)
    
    # Step 2: Find vertices with odd degrees in the MST
    odd_degree_nodes = [v for v, d in mst.degree() if d % 2 == 1]
    
    # Step 3: Find a minimum-weight matching on the odd-degree vertices
    subgraph = G.subgraph(odd_degree_nodes)
    matching = max_weight_matching(subgraph, maxcardinality=True, weight='weight')

    # Step 4: Combine the MST and matching edges
    multigraph = nx.MultiGraph(mst)
    multigraph.add_edges_from(matching)

    # Step 5: Find an Eulerian circuit in the multigraph
    eulerian_circuit = list(nx.eulerian_circuit(multigraph))

    # Step 6: Convert the Eulerian circuit to a Hamiltonian path (remove repeated vertices)
    visited = set()
    hamiltonian_path = []
    for u, v in eulerian_circuit:
        if u not in visited:
            visited.add(u)
            hamiltonian_path.append(u)
    hamiltonian_path.append(hamiltonian_path[0])  # Return to the starting node

    return hamiltonian_path
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
def solve_nn(distance_matrix):
    num_nodes = len(distance_matrix)
    visited = [False] * num_nodes
    path = [0]  # Start from the first node
    visited[0] = True
    
    for _ in range(1, num_nodes):
        last = path[-1]
        next_node = None
        min_distance = float('inf')
        for i in range(num_nodes):
            if not visited[i] and distance_matrix[last][i] < min_distance:
                next_node = i
                min_distance = distance_matrix[last][i]
        path.append(next_node)
        visited[next_node] = True
    
    path.append(0)  # Return to the start to complete the tour
    return path

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

def print_total_distance(graph, destinations, best_path):
    total_distance = 0
    
    # Iterate through the path and calculate the total distance
    for i in range(len(best_path) - 1):
        start_node = str(destinations[best_path[i]])
        end_node = str(destinations[best_path[i + 1]])
        
        try:
            # Find the shortest path based on distance (weight='length' assuming distance is stored in this attribute)
            sub_path = nx.shortest_path(graph, source=start_node, target=end_node, weight='length')
            distance = nx.shortest_path_length(graph, source=start_node, target=end_node, weight='length')
            
            total_distance += distance
            print(f"Distance from {start_node} to {end_node}: {distance:.2f} meters, Path: {sub_path}")
        
        except nx.NetworkXNoPath:
            # Handle the case where no path exists
            print(f"No path exists from {start_node} to {end_node} due to uni-directional edges.")
    
    # Print the total distance traveled
    print(f"Total distance traveled: {total_distance:.2f} meters")

def main():
    
    destinations = [386, 343, 362, 368, 317, 318, 404, 399, 425, 420, 437, 82, 80, 93, 121, 116, 127, 75, 71, 
                    185, 27, 25, 31, 49, 301, 8, 289, 199, 42, 225, 228, 239, 261, 257, 56]
    
    graph_file = '/maps/Competition_track_graph_modified_new.graphml'
    current_dir = os.path.dirname(os.path.realpath(__file__))
    graph = nx.read_graphml(current_dir + graph_file)
    
    start_x, start_y = 0.5, 13.786-3.9
    closest_node = find_closest_node(graph, start_x, start_y)
    closest_node_int = int(closest_node)
    if closest_node_int not in destinations:
        destinations.insert(0, closest_node_int)
        
    coordinates = get_node_coordinates(graph, destinations)
    #invert y coordinates
    coordinates = [(x, 13.786-y) for x, y in coordinates]
    
    distance_matrix = create_graph_distance_matrix(graph, destinations)
    hop_matrix = create_graph_hop_matrix(graph, destinations)
    
    print("solving...")
    # best_path = solve_tsp_with_ortools(distance_matrix)
    # best_path = solve_christofides_tsp(distance_matrix)
    # best_path = solve_nn(distance_matrix)
    # best_path = solve_nn(hop_matrix)
    best_path = solve_christofides_tsp(hop_matrix)
    
    if best_path:
        total_distance = 0
        print("Best Path (order of node indices):", [destinations[i] for i in best_path])
        # Print the distances between consecutive nodes and the total distance
        print_total_distance(graph, destinations, best_path)
        plot_path_on_map(graph, coordinates, destinations, best_path, current_dir + '/maps/Track.png')
    else:
        print("No solution found!")

if __name__ == "__main__":
    main()
