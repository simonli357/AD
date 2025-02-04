import networkx as nx
import numpy as np
import os
import yaml

class GlobalPlanner:
    def __init__(self):
        self.hw_safety_offset = 0.05#0.1
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_modified_new.graphml')
        self.pos = {}
        self.attribute = {}
        for node, data in self.G.nodes(data=True):
            x = data.get('x', 0.0) 
            y = data.get('y', 0.0)
            self.pos[node] = (x, 13.786 - y) # 13.786 is the height of the map
            self.attribute[node] = data.get('new_attribute', 0)
        
        self.wp_x = []
        self.wp_y = []
                
        current_path = os.path.dirname(os.path.realpath(__file__))
        with open(os.path.join(current_path + '/../../planning/scripts/config/paths.yaml'), 'r') as stream:
            data1 = yaml.safe_load(stream)
            destinations = data1['destinations']
            
        return

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

if __name__ == "__main__":
    planner = GlobalPlanner()
