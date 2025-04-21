import sqlite3
import networkx as nx
import io


class GraphQueries():
    def __init__(self, db_path):
        super(GraphQueries, self).__init__()
        self.db_path = db_path

    def fetch_graph(self):
        try:
            conn = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True, timeout=5.0)
            cur = conn.cursor()
            cur.execute("SELECT value FROM graph WHERE name = ?", ("graph",))
            row = cur.fetchone()
            conn.close()
            if not row or row[0] is None:
                return None
            graphml_str = row[0]
            buf = io.StringIO(graphml_str)
            G = nx.read_graphml(buf)
            return G
        except Exception:
            return None
