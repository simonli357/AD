import networkx as nx
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
from ortools.constraint_solver import pywrapcp, routing_enums_pb2
import os
import random
import time
import math
from networkx.algorithms.matching import max_weight_matching

def plot_path_on_map(graph, coordinates, destinations, path, map_image_path):
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

def print_total_distance(graph, best_path):
    total_distance = 0

    # Iterate through the path and calculate the total distance
    for i in range(len(best_path) - 1):
        start_node = str(best_path[i])
        end_node = str(best_path[i + 1])
        
        try:
            # Get the shortest path between start_node and end_node
            sub_path = nx.shortest_path(graph, source=start_node, target=end_node)
            
            # Calculate the distance for the sub-path
            sub_path_distance = 0
            for j in range(len(sub_path) - 1):
                node_a = str(sub_path[j])
                node_b = str(sub_path[j + 1])
                
                # Extract the x, y coordinates for consecutive nodes
                x1, y1 = graph.nodes[node_a]['x'], graph.nodes[node_a]['y']
                x2, y2 = graph.nodes[node_b]['x'], graph.nodes[node_b]['y']
                
                # Calculate the Euclidean distance between the nodes
                distance = math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2)
                sub_path_distance += distance

            total_distance += sub_path_distance
            # print(f"Distance from {start_node} to {end_node} (via path {sub_path}): {sub_path_distance:.2f} meters")
        
        except nx.NetworkXNoPath:
            # Handle the case where no path exists
            print(f"No path exists from {start_node} to {end_node} due to uni-directional edges.")
        except KeyError as e:
            print(f"Missing coordinate data for nodes: {e}")
    
    # Print the total distance traveled
    print(f"Total distance traveled: {total_distance:.2f} meters")

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
    distance_matrix = np.full((num_nodes, num_nodes), float('inf'))  # Initialize with infinity (no path)

    for i in range(num_nodes):
        for j in range(num_nodes):
            if i == j:
                distance_matrix[i][j] = 0  # No distance for the same node
            else:
                try:
                    # Compute the shortest path considering uni-directional edges
                    distance = nx.shortest_path_length(graph, str(destinations[i]), str(destinations[j]), weight='length')
                    distance_matrix[i][j] = distance
                except nx.NetworkXNoPath:
                    # If no path exists, leave the entry as infinity
                    print(f"No path exists between {destinations[i]} and {destinations[j]}")
                    pass
    return distance_matrix

def solve_nn(distance_matrix, destinations):
    num_nodes = len(distance_matrix)
    visited = [False] * num_nodes
    path = [0]  # Start from the first node
    visited[0] = True
    total_distance = 0  # Initialize the total distance
    
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
        total_distance += min_distance  # Add the distance to the total
    
    # Return to the start to complete the tour
    # path.append(0)
    # total_distance += distance_matrix[path[-2]][path[-1]]  # Add the distance to return to start
    
    # convert final path to actual node indices
    path = [destinations[i] for i in path]
    return path, total_distance

# def simulated_annealing_tsp(distance_matrix, initial_temp=1000, cooling_rate=0.995, stopping_temp=1e-3, max_iter=1000):
def simulated_annealing_tsp(distance_matrix, destinations, initial_temp=1000, cooling_rate=0.999, stopping_temp=1e-5, max_iter=5000):
    """
    Solve the TSP using Simulated Annealing.

    Args:
        distance_matrix (2D array): A square matrix where distance_matrix[i][j]
                                    is the distance from node i to node j.
        initial_temp (float): The initial temperature for the algorithm.
        cooling_rate (float): The rate at which the temperature decreases.
        stopping_temp (float): The temperature at which the algorithm stops.
        max_iter (int): Maximum number of iterations at each temperature.

    Returns:
        tuple: The best path and its cost.
    """
    def calculate_total_distance(path):
        """Calculate the total distance of a path."""
        return sum(distance_matrix[path[i]][path[i + 1]] for i in range(len(path) - 1))

    # Initialize with a random solution that keeps the first node fixed
    num_nodes = len(distance_matrix)
    current_path = [0] + random.sample(range(1, num_nodes), num_nodes - 1)
    current_distance = calculate_total_distance(current_path)

    # Set initial conditions
    best_path = list(current_path)
    best_distance = current_distance
    temperature = initial_temp

    while temperature > stopping_temp:
        for _ in range(max_iter):
            # Generate a neighbor by swapping two cities, avoiding the fixed start/end
            new_path = list(current_path)
            i, j = random.sample(range(1, num_nodes), 2)  # Only swap nodes from 1 to num_nodes-1
            new_path[i], new_path[j] = new_path[j], new_path[i]
            new_distance = calculate_total_distance(new_path)

            # Accept new path with a probability
            if new_distance < current_distance or random.random() < np.exp((current_distance - new_distance) / temperature):
                current_path = new_path
                current_distance = new_distance

                # Update best solution
                if current_distance < best_distance:
                    best_path = list(current_path)
                    best_distance = current_distance

        # Cool down the temperature
        temperature *= cooling_rate
    
    # convert final path to actual node indices
    best_path = [destinations.index(node) for node in best_path]
    return best_path, best_distance

from tqdm import tqdm
from functools import lru_cache

def reduce_with_fixed_sequences(distance_matrix, destinations, fixed_sequences):
    """
    Simplify the problem by treating fixed sequences as single entities.

    Parameters:
    - distance_matrix: The original distance matrix.
    - destinations: The list of destination IDs.
    - fixed_sequences: A list of fixed sequences of destinations.

    Returns:
    - reduced_distance_matrix: A new distance matrix with fixed sequences collapsed.
    - reduced_destinations: The new list of destinations (super-nodes for fixed sequences).
    - fixed_sequence_mapping: A mapping from reduced nodes to the original destinations.
    """
    destination_to_index = {dest: i for i, dest in enumerate(destinations)}

    # Map fixed sequences to indices
    fixed_sequences_mapped = [
        [destination_to_index[dest] for dest in seq] for seq in fixed_sequences
    ]

    # Create a mapping for the reduced problem
    fixed_sequence_mapping = {}
    reduced_destinations = []
    new_indices = {}  # Map old indices to reduced indices

    # Add super-nodes for fixed sequences
    for i, seq in enumerate(fixed_sequences_mapped):
        super_node = f"seq_{i}"
        reduced_destinations.append(super_node)
        fixed_sequence_mapping[super_node] = seq
        for index in seq:
            new_indices[index] = super_node

    # Add remaining destinations that are not part of any fixed sequence
    for i, dest in enumerate(destinations):
        if i not in new_indices:
            reduced_destinations.append(dest)
            new_indices[i] = dest

    # Build the reduced distance matrix
    size = len(reduced_destinations)
    reduced_distance_matrix = np.full((size, size), float("inf"))

    for i, node_i in enumerate(reduced_destinations):
        for j, node_j in enumerate(reduced_destinations):
            if i == j:
                reduced_distance_matrix[i][j] = 0
            elif isinstance(node_i, str) and isinstance(node_j, str):
                # Distance between two fixed sequences
                seq_i = fixed_sequence_mapping[node_i]
                seq_j = fixed_sequence_mapping[node_j]
                reduced_distance_matrix[i][j] = min(
                    distance_matrix[seq_i[-1]][seq_j[0]],  # End of seq_i to start of seq_j
                    distance_matrix[seq_i[0]][seq_j[-1]],  # Start of seq_i to end of seq_j
                )
            elif isinstance(node_i, str):
                # Distance from fixed sequence to a normal node
                seq_i = fixed_sequence_mapping[node_i]
                reduced_distance_matrix[i][j] = min(
                    distance_matrix[seq_i[-1]][destination_to_index[node_j]],
                    distance_matrix[seq_i[0]][destination_to_index[node_j]],
                )
            elif isinstance(node_j, str):
                # Distance from a normal node to a fixed sequence
                seq_j = fixed_sequence_mapping[node_j]
                reduced_distance_matrix[i][j] = min(
                    distance_matrix[destination_to_index[node_i]][seq_j[0]],
                    distance_matrix[destination_to_index[node_i]][seq_j[-1]],
                )
            else:
                # Distance between two normal nodes
                reduced_distance_matrix[i][j] = distance_matrix[
                    destination_to_index[node_i]
                ][destination_to_index[node_j]]

    return reduced_distance_matrix, reduced_destinations, fixed_sequence_mapping

def max_destinations_with_reduced_problem(
    reduced_distance_matrix, max_distance, reduced_destinations, fixed_sequence_mapping
):
    """
    Solve the reduced problem using dynamic programming with progress tracking.

    Parameters:
    - reduced_distance_matrix: Distance matrix for the reduced problem.
    - max_distance: Maximum allowable distance.
    - reduced_destinations: Reduced list of destinations.
    - fixed_sequence_mapping: Mapping from reduced nodes to the original destinations.

    Returns:
    - max_destinations: Maximum number of destinations visited.
    - best_path: The corresponding path (in terms of destination IDs).
    """
    num_nodes = len(reduced_distance_matrix)

    # Initialize tqdm progress bar
    total_states = 2 ** num_nodes  # Total number of states in DP
    progress_bar = tqdm(total=total_states, desc="Processing states", unit="state")
    visited_states = set()  # Track unique states for progress updates

    @lru_cache(None)
    def dp(current, visited, remaining_distance):
        # Only update progress for new unique states
        if (current, visited) not in visited_states:
            visited_states.add((current, visited))
            progress_bar.update(1)

        max_count = 0
        best_path = []

        for next_node in range(num_nodes):
            if (visited & (1 << next_node)) == 0:  # If not yet visited
                distance = reduced_distance_matrix[current][next_node]
                if remaining_distance >= distance:  # If within distance limit
                    # Recurse to the next state
                    count, path = dp(
                        next_node,
                        visited | (1 << next_node),  # Mark next_node as visited
                        remaining_distance - distance,
                    )
                    count += len(
                        fixed_sequence_mapping.get(reduced_destinations[next_node], [next_node])
                    )  # Add the size of the fixed sequence or single node

                    if count > max_count:
                        max_count = count
                        best_path = [next_node] + path

        return max_count, best_path

    # Start from the first node, with no nodes visited except the start
    start_node = 0
    initial_visited = 1 << start_node  # Mark start node as visited
    max_destinations, best_path_indices = dp(start_node, initial_visited, max_distance)

    # Close the progress bar
    progress_bar.close()

    # Resolve the reduced path back to the original destinations
    resolved_path = []
    for node_index in [start_node] + best_path_indices:
        if reduced_destinations[node_index] in fixed_sequence_mapping:
            # Expand the fixed sequence
            resolved_path.extend(fixed_sequence_mapping[reduced_destinations[node_index]])
        else:
            # Add the single node
            resolved_path.append(reduced_destinations[node_index])

    # Validate the resolved path contains only valid destinations
    resolved_path = [dest for dest in resolved_path if dest in reduced_destinations]

    # Verify the reconstructed path matches the count
    if len(resolved_path) != max_destinations:
        raise ValueError(
            f"Mismatch: Max destinations ({max_destinations}) != Path length ({len(resolved_path)})"
        )

    return max_destinations, resolved_path

def save_path_to_file(path, filename):
    """
    Save the given path to a text file.

    Args:
        path (list): The list of node indices representing the path.
        filename (str): The name of the file to save the path.
    """
    current_dir = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(current_dir, filename)
    with open(filename, 'w') as f:
        for node in path:
            f.write(f"{node}\n")
    print(f"Path saved to {filename}")
def load_path_from_file(filename):
    """
    Load a path from a text file, stopping at the first empty line.

    Args:
        filename (str): The name of the file to load the path from.

    Returns:
        list: The list of node indices representing the path.
    """
    current_dir = os.path.dirname(os.path.realpath(__file__))
    filename = os.path.join(current_dir, filename)
    path = []
    with open(filename, 'r') as f:
        for line in f:
            stripped_line = line.strip()
            if not stripped_line:  # Stop at the first empty line
                break
            path.append(int(stripped_line))
    print(f"Path loaded from {filename}")
    return path
def main():
    
    destinations = [386, 343, 362, 368, 317, 318, 404, 399, 425, 420, 437, 82, 80, 93, 121, 116, 127, 75, 71, 
                    185, 27, 25, 31, 29, 301, 8, 289, 199, 42, 225, 228, 239, 261, 257, 56]
    fixed_sequences = [[399,425,437], [318, 56], [225, 228,239,257], [116,121], [127,75], [420,404], [368,386],[343,362]]
    max_distance = 150
    # destinations = [386, 362, 317, 404, 425, 82, 80, 93, 121, 75, 71, 
    #                 185, 27, 25, 31, 29, 301, 8, 289, 199, 42, 239, 261, 257, 56]
    
    graph_file = '/maps/Competition_track_graph_modified_new.graphml'
    current_dir = os.path.dirname(os.path.realpath(__file__))
    graph = nx.read_graphml(current_dir + graph_file)
    
    closest_node_int = 145
    if closest_node_int not in destinations:
        destinations.insert(0, closest_node_int)
        
    coordinates = get_node_coordinates(graph, destinations)
    #invert y coordinates
    coordinates = [(x, 13.786-y) for x, y in coordinates]
    
    # distance_matrix = create_graph_distance_matrix(graph, destinations)
    # hop_matrix = create_graph_hop_matrix(graph, destinations)
    
    print("solving...")
    t1 = time.time()
    # best_path, best_distance = solve_nn(distance_matrix, destinations)
    # best_path, best_distance = solve_nn(hop_matrix, destinations)
    # best_path, best_distance = simulated_annealing_tsp(distance_matrix, destinations)
    # reduced_distance_matrix, reduced_destinations, fixed_sequence_mapping = reduce_with_fixed_sequences(
    #     distance_matrix, destinations, fixed_sequences
    # )
    # max_destinations, best_path = max_destinations_with_reduced_problem(
    #     reduced_distance_matrix, max_distance, reduced_destinations, fixed_sequence_mapping
    # )
    # print(f"Maximum number of destinations within {max_distance} distance: {max_destinations}")
    # print(f"Best path: {best_path}")
    # print("time:", time.time()-t1)
    # print("best distance:", best_distance)
    
    # save_path_to_file(best_path, "paths/dp1.txt")
    
    best_path = load_path_from_file("paths/manual1.txt")
    if best_path:
        print("Best Path (order of node indices):", best_path)
        # Print the distances between consecutive nodes and the total distance
        print_total_distance(graph, best_path)
        plot_path_on_map(graph, coordinates, destinations, best_path, current_dir + '/maps/Track.png')
    else:
        print("No solution found!")

if __name__ == "__main__":
    main()
