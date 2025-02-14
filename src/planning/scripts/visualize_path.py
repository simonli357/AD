import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import yaml
import os
import numpy as np
from global_planner import GlobalPlanner

class PathVisualizer:
    def __init__(self):
        self.planner = GlobalPlanner()
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        
        self.track_img = mpimg.imread(os.path.join(self.current_dir, 'maps/Track.png'))
        self.track_img = np.flipud(self.track_img)
        
        self.track_width = 20.696
        self.track_height = 13.656
        
        self.min_x = 0.0
        self.max_x = self.track_width
        self.min_y = 0.0
        self.max_y = self.track_height

    def load_runs(self, yaml_path):
        with open(yaml_path, 'r') as f:
            return yaml.safe_load(f)

    def plot_run(self, run_name, sequence):
        plt.figure(figsize=(12, 8))
        
        plt.imshow(self.track_img, 
                  extent=[self.min_x, self.max_x, self.max_y, self.min_y],  # Flip Y-axis
                  aspect='equal', 
                  alpha=0.7)
        
        plt.xlim(self.min_x, self.max_x)
        plt.ylim(self.min_y, self.max_y)
        
        full_path_x = []
        full_path_y = []
        
        sequence = [str(node) for node in sequence]
        
        for i in range(len(sequence)-1):
            start_node = sequence[i]
            end_node = sequence[i+1]
            
            path_points, _, _ = self.planner.plan_path(start_node, end_node)
            
            if full_path_x:
                full_path_x.extend(path_points[0][1:])
                full_path_y.extend(path_points[1][1:])
            else:
                full_path_x.extend(path_points[0])
                full_path_y.extend(path_points[1])
        
        plt.plot(full_path_x, full_path_y, 'b-', linewidth=1.5, alpha=0.8, label='Path')
        
        for order, node in enumerate(sequence, 1):
            if node in self.planner.pos:
                x, y = self.planner.pos[node]
                plt.plot(x, y, 'o', markersize=10, color='#FF4500', markeredgecolor='black')
                plt.text(x, y, str(order), 
                        ha='center', va='center', 
                        fontsize=9, fontweight='bold',
                        color='white', 
                        bbox=dict(facecolor='black', alpha=0.7, boxstyle='round'))
        
        total_distance = self.planner.get_total_distance(sequence)
        plt.xlabel('X Position (meters)')
        plt.ylabel('Y Position (meters)')
        plt.title(f'Path Visualization: {run_name}\nTotal Destinations: {len(sequence)-1} | Total Distance: {total_distance:.2f} meters')
        plt.grid(True, alpha=0.3)
        plt.legend(loc='upper right')
        plt.tight_layout()
        plt.show()

def main():
    visualizer = PathVisualizer()
    
    yaml_path = os.path.join(visualizer.current_dir, 'config/runs_mod2_custom.yaml')
    runs = visualizer.load_runs(yaml_path)
    
    # custom_run = [56, 54] # Avram 10m
    # custom_run = [368, 341] # Laneless outer 14m
    # custom_run = [342, 335] # Laneless inner 11m
    # visualizer.plot_run('custom_run', custom_run)
    selected_run = 'run221'
    
    if selected_run in runs:
        print(f"Visualizing {selected_run}...")
        start = selected_run[3:]
        start = int(start)
        runs[selected_run].insert(0, start)
        visualizer.plot_run(selected_run, runs[selected_run])
    else:
        print(f"Run {selected_run} not found in YAML file")

if __name__ == "__main__":
    main()