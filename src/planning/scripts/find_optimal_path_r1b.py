import networkx as nx
import numpy as np
import os
import yaml
import math

class GlobalPlanner:
    def __init__(self):
        self.hw_safety_offset = 0.05
        self.current_dir = os.path.dirname(os.path.realpath(__file__))
        self.G = nx.read_graphml(self.current_dir + '/maps/Competition_track_graph_modified_new.graphml')
        self.pos = {}
        self.attribute = {}
        for node, data in self.G.nodes(data=True):
            x = data.get('x', 0.0)
            y = data.get('y', 0.0)
            self.pos[node] = (x, 13.786 - y)
            self.attribute[node] = data.get('new_attribute', 0)
        
        # Compute edge weights based on Euclidean distance
        for u, v in self.G.edges():
            if u in self.pos and v in self.pos:
                x1, y1 = self.pos[u]
                x2, y2 = self.pos[v]
                distance = math.sqrt((x2 - x1)**2 + (y2 - y1)**2)
                self.G[u][v]['weight'] = distance
            else:
                self.G[u][v]['weight'] = float('inf')
        
        # Define destinations from the problem statement, converting to strings
        self.destinations = ['112', '386', '343', '362', '368', '317', '318', '404', '399', 
                             '425', '420', '437', '82', '80', '93', '121', '116', '127', '75', 
                             '71', '185', '27', '25', '31', '29', '301', '8', '289', '199', 
                             '42', '225', '228', '239', '261', '257', '56']
        
        # Precompute distance matrix between all pairs of destinations
        self.distance_matrix = {}
        for start in self.destinations:
            self.distance_matrix[start] = {}
            for end in self.destinations:
                if start == end:
                    self.distance_matrix[start][end] = 0.0
                else:
                    try:
                        length = nx.dijkstra_path_length(self.G, start, end, weight='weight')
                        self.distance_matrix[start][end] = length
                    except nx.NetworkXNoPath:
                        self.distance_matrix[start][end] = float('inf')

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

    def find_optimal_sequence(self):
        start_node = '112'
        if start_node not in self.destinations:
            return [], 0.0
        
        remaining_budget = 150.0
        current_node = start_node
        visited = set([current_node])
        path = [current_node]
        total_distance = 0.0

        # Create a list of destination candidates excluding the start node
        candidates = [d for d in self.destinations if d != start_node]
        
        while True:
            best_next = None
            best_accessible_count = -1
            min_distance = float('inf')
            
            eligible_candidates = []
            # First pass: filter candidates within remaining budget
            for candidate in candidates:
                if candidate in visited:
                    continue
                dist = self.distance_matrix[current_node].get(candidate, float('inf'))
                if total_distance + dist > remaining_budget:
                    continue
                eligible_candidates.append(candidate)
            
            # If no eligible candidates, break
            if not eligible_candidates:
                break
            
            # Second pass: calculate accessible count for each eligible candidate
            candidate_scores = {}
            for candidate in eligible_candidates:
                dist = self.distance_matrix[current_node][candidate]
                remaining_after = remaining_budget - (total_distance + dist)
                accessible_count = 0
                for node in candidates:
                    if node == candidate or node in visited:
                        continue
                    node_dist = self.distance_matrix[candidate].get(node, float('inf'))
                    if node_dist <= remaining_after:
                        accessible_count += 1
                candidate_scores[candidate] = (accessible_count, dist)
            
            # Find the candidate with the highest accessible count, then smallest distance
            max_accessible = max([score[0] for score in candidate_scores.values()], default=-1)
            best_candidates = [cand for cand, (acc, dist) in candidate_scores.items() if acc == max_accessible]
            
            if max_accessible == 0:
                # All have 0 accessible, pick the closest
                min_dist = min([candidate_scores[cand][1] for cand in eligible_candidates])
                best_next = next(cand for cand in eligible_candidates if candidate_scores[cand][1] == min_dist)
            else:
                # Among candidates with max accessible count, pick the closest
                min_dist = min([candidate_scores[cand][1] for cand in best_candidates])
                best_next = next(cand for cand in best_candidates if candidate_scores[cand][1] == min_dist)
            
            if best_next:
                dist = self.distance_matrix[current_node][best_next]
                path.append(best_next)
                visited.add(best_next)
                total_distance += dist
                current_node = best_next
                candidates.remove(best_next)
                
                if total_distance >= remaining_budget:
                    break
            else:
                break
        
        return path, total_distance

if __name__ == "__main__":
    planner = GlobalPlanner()
    optimal_path, total_dist = planner.find_optimal_sequence()
    print(f"Optimal sequence of destinations: {optimal_path}")
    print(f"Total distance traveled: {total_dist} meters")
    print(f"Number of destinations visited: {len(optimal_path)}")