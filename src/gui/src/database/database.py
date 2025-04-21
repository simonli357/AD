import os

from .queries.cam_queries import CamQueries


class Database():
    def __init__(self):
        super(Database, self).__init__()
        current_dir = os.path.dirname(os.path.abspath(__file__))
        src = os.path.dirname(os.path.dirname(os.path.dirname(current_dir)))
        db_path = os.path.join(src, 'persistence', 'share', 'database.db')

        self.cam_queries = CamQueries(db_path)
