import numpy as np

def analyze_waypoints_with_outliers(file_path, threshold=1.5):
    # Load the data from the text file
    data = np.loadtxt(file_path)
    x, y, yaw = data[:, 0], data[:, 1], data[:, 2]
    
    # Compute distances between consecutive waypoints
    distances = np.sqrt(np.diff(x)**2 + np.diff(y)**2)
    
    # Calculate statistics
    avg_distance = np.mean(distances)
    std_distance = np.std(distances)
    max_distance = np.max(distances)
    min_distance = np.min(distances)
    median_distance = np.median(distances)
    
    # Total distance and waypoint density
    total_distance = np.sum(distances)
    waypoint_density = len(x) / total_distance if total_distance > 0 else np.inf
    
    # Identify outliers based on the median
    deviation = np.abs(distances - median_distance)
    mad = np.median(deviation)  # Median Absolute Deviation (MAD)
    outlier_threshold = threshold * mad  # Threshold for identifying outliers
    outlier_threshold = 0.005
    print("Outlier Threshold:", outlier_threshold)
    outliers = distances[deviation > outlier_threshold]
    outlier_indices = np.where(deviation > outlier_threshold)[0] + 1  # Indices of waypoints after outliers

    # Print results
    print("Waypoint Analysis:")
    print(f"Average Distance: {avg_distance:.4f}")
    print(f"Standard Deviation: {std_distance:.4f}")
    print(f"Median Distance: {median_distance:.4f}")
    print(f"Maximum Distance: {max_distance:.4f}")
    print(f"Minimum Distance: {min_distance:.4f}")
    print(f"Total Distance: {total_distance:.4f}")
    print(f"Waypoint Density: {waypoint_density:.4f} waypoints per unit distance")
    print(f"Number of Outliers: {len(outliers)}")
    # if len(outliers) > 0:
    #     print(f"Outliers (too far from median): {outliers}")
    #     print(f"Outlier Indices: {outlier_indices}")
    
    return {
        "average_distance": avg_distance,
        "std_distance": std_distance,
        "median_distance": median_distance,
        "max_distance": max_distance,
        "min_distance": min_distance,
        "total_distance": total_distance,
        "waypoint_density": waypoint_density,
        "outliers": outliers,
        "outlier_indices": outlier_indices,
    }

file_path = 'state_refs.txt'
stats = analyze_waypoints_with_outliers(file_path)
