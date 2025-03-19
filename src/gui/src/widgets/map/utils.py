import os


class MapUtils:
    def __init__(self):
        current_dir = os.path.dirname(os.path.abspath(__file__))
        self.graphml = os.path.join(current_dir, 'graph.graphml')
