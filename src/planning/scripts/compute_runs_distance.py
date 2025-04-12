import os
import yaml
from global_planner import GlobalPlanner

def main():
    planner = GlobalPlanner()
    current_dir = os.path.dirname(os.path.realpath(__file__))

    # Load starting points
    starting_points_path = os.path.join(current_dir, 'config/starting_points.yaml')
    with open(starting_points_path, 'r') as f:
        starting_points = yaml.safe_load(f)
    starting_points = [str(s) for s in starting_points]

    # Load runs
    runs_yaml_path = os.path.join(current_dir, 'config/runs.yaml')  # Adjust if using a different one
    with open(runs_yaml_path, 'r') as f:
        runs_data = yaml.safe_load(f)

    total_overall_distance = 0.0
    print("=== Run Distance Summary ===")

    for start in starting_points:
        run_key = f"run{start}"
        if run_key not in runs_data:
            print(f"{run_key}: Not found in runs.yaml")
            continue

        path = [start] + [str(node) for node in runs_data[run_key]]
        total_distance = planner.get_total_distance(path)
        total_overall_distance += total_distance

        print(f"{run_key}: Distance = {total_distance:.2f} meters | Destinations = {len(path)-1}")

    print("\n=== Summary ===")
    print(f"Total Distance Across All Runs: {total_overall_distance:.2f} meters")

if __name__ == "__main__":
    main()
