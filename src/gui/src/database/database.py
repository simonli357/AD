import os

from .queries.cam_queries import CamQueries
from .queries.graph_queries import GraphQueries


class Database():
    def __init__(self):
        super(Database, self).__init__()
        current_dir = os.path.dirname(os.path.abspath(__file__))
        src = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
        db_dir = os.path.join(src, 'persistence', 'share')
        db_path = os.path.join(db_dir, 'database.db')

        os.makedirs(db_dir, exist_ok=True)
        open(db_path, 'a').close()

        self.cam_queries = CamQueries(db_path)
        self.graph_queries = GraphQueries(db_path)
