import numpy as np

def analyze_yaw_outliers(waypoints, yaw_threshold=1.5):
    """
    Analyze yaw changes and find outliers with abrupt changes.
    
    Parameters:
    - waypoints: A numpy array where each row is [x, y, yaw].
    - yaw_threshold: The multiplier for identifying outliers (default: 1.5).
    
    Returns:
    - A dictionary containing yaw change statistics and indices of outliers.
    """
    yaw = waypoints[:, 2]  # Extract yaw values

    # Compute differences in yaw between consecutive waypoints
    yaw_diff = np.diff(yaw)

    # Normalize yaw differences to the range [-pi, pi] for correct angular difference
    yaw_diff = (yaw_diff + np.pi) % (2 * np.pi) - np.pi

    # Statistics
    avg_yaw_change = np.mean(np.abs(yaw_diff))
    median_yaw_change = np.median(np.abs(yaw_diff))
    std_yaw_change = np.std(yaw_diff)

    # Median Absolute Deviation (MAD) for robust outlier detection
    mad = np.median(np.abs(yaw_diff - median_yaw_change))
    outlier_threshold = yaw_threshold * mad
    print("Outlier Threshold (based on MAD):", outlier_threshold)
    outlier_threshold = 1
    outlier_indices = np.where(np.abs(yaw_diff) > outlier_threshold)[0] + 1  # +1 for consecutive index

    # Outliers (yaw change values that exceed the threshold)
    outlier_values = yaw_diff[np.abs(yaw_diff) > outlier_threshold]

    # Print results
    print("Yaw Analysis:")
    print(f"Average Yaw Change: {avg_yaw_change:.4f}")
    print(f"Median Yaw Change: {median_yaw_change:.4f}")
    print(f"Standard Deviation: {std_yaw_change:.4f}")
    print(f"Outlier Threshold (based on MAD): {outlier_threshold:.4f}")
    print(f"Number of Outliers: {len(outlier_values)}")
    if len(outlier_values) > 0:
        print(f"Outlier Indices: {outlier_indices}")
        print(f"Outlier Yaw Changes: {outlier_values}")

    return {
        "avg_yaw_change": avg_yaw_change,
        "median_yaw_change": median_yaw_change,
        "std_yaw_change": std_yaw_change,
        "outlier_threshold": outlier_threshold,
        "outlier_indices": outlier_indices,
        "outlier_values": outlier_values,
    }

# Load waypoints from a text file
waypoints = np.loadtxt("state_refs.txt")

yaw_outliers = analyze_yaw_outliers(waypoints)
