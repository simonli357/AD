from typing import List, Tuple

import os
import networkx as nx


class MapUtils:
    def __init__(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        graphml = os.path.join(current_dir, 'graph.graphml')
        self.graph = nx.read_graphml(graphml)
        self.destination_ids = [
            386, 343, 362, 368, 317, 318, 404, 399, 425, 420, 437, 82, 80, 93, 121, 116, 127, 75, 71,
            185, 27, 25, 31, 29, 301, 8, 289, 199, 42, 225, 228, 239, 261, 257, 56,
        ]

    def get_all_nodes(self) -> List[Tuple[int, float, float]]:
        return [
            (
                int(node_id),
                float(node_data["x"]),
                float(node_data["y"])
            )
            for node_id, node_data in self.graph.nodes(data=True)
        ]

    def get_destination_nodes(self) -> List[Tuple[int, float, float]]:
        return [
            (
                int(node_id),
                float(node_data["x"]),
                float(node_data["y"])
            )
            for node_id, node_data in self.graph.nodes(data=True) if int(node_id) in self.destination_ids
        ]
