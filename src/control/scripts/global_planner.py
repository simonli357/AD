import networkx as nx
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as alex
import os

class GlobalPlanner:
    def __init__(self):
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_mod.graphml')
        # self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph.graphml')
        self.pos = {}
        self.intersection = {}
        for node, data in self.G.nodes(data=True):
            # print("node: ", node)
            self.pos[node] = (data['x'], 15-data['y'])
            self.intersection[node] = data['intersection']
        self.wp_x = []
        self.wp_y = []
        self.place_names = {}

        self.place_names = {
            "parallel_parking1": 180,
            "parallel_parking2": 182,
            "front_parking1": 160,
            "front_parking2": 162,
            "highway1_start": 49,
            "highway1_mid": 326,
            "highway2_start": 343,
            "highway2_mid": 357,
            "curved_path": 426,
            "start": 86,
            "end": 85,
            "speedrun2": 467,
            "speedrun3": 302,
            "speedrun4": 264,
            "speedrun5": 81
        }
        
    def plan_path(self, start, end):
        path = nx.dijkstra_path(self.G, source=str(start), target=str(end))
        path_edges = [(path[i], path[i + 1]) for i in range(len(path) - 1)]
        wp_x = []
        wp_y = []
        for node in path:
            if not self.intersection[node]:
                x, y = self.pos[node]
                wp_x.append(x)
                wp_y.append(y)
            else:
                print("intersection: ", node)
        return alex.array([wp_x, wp_y]), path_edges
    def illustrate_path(self, start, end):
        _, path_edges = self.plan_path(start, end)
        img = mpimg.imread(self.current_dir+'/maps/map.png')
        # img = mpimg.imread(self.current_dir+'/maps/Competition_track_graph.png')
        print("img: ", img.shape)
        # Create the plot
        fig, ax = plt.subplots()

        # Display the image
        ax.imshow(img, extent=[0, 15, 0, 15]) 

        # Draw the graph
        nx.draw(self.G, self.pos, ax=ax, with_labels=True, node_size=20, node_color='r', font_size=6)
        # Highlight the path
        nx.draw_networkx_edges(self.G, self.pos, edgelist=path_edges, edge_color='g', width=2)
        plt.show()
  
if __name__ == "__main__":
    planner = GlobalPlanner()
    planner.illustrate_path(86, 467)
    