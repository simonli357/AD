from typing import List, Tuple

import os
import networkx as nx


class MapUtils:
    def __init__(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        graphml = os.path.join(current_dir, 'graph.graphml')
        self.graph = nx.read_graphml(graphml)

    def get_all_nodes(self) -> List[Tuple[int, float, float]]:
        return [
            (
                int(node_id),
                float(node_data["x"]),
                float(node_data["y"])
            )
            for node_id, node_data in self.graph.nodes(data=True)
        ]
