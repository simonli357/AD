import sqlite3


class CamQueries():
    def __init__(self, db_path):
        super(CamQueries, self).__init__()
        self.db_path = db_path

    def fetch_camera_sim_params(self):
        conn = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True, timeout=5.0)
        cur = conn.cursor()
        cur.execute("""
            SELECT name, value
              FROM camera_params_sim
            ORDER BY CASE name
              WHEN 'fx' THEN 0
              WHEN 'fy' THEN 1
              WHEN 'cx' THEN 2
              WHEN 'cy' THEN 3
            END;
        """)
        rows = cur.fetchall()
        conn.close()
        return [value for (_, value) in rows]

    def fetch_camera_real_params(self):
        conn = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True, timeout=5.0)
        cur = conn.cursor()
        cur.execute("""
            SELECT name, value
              FROM camera_params_real
            ORDER BY CASE name
              WHEN 'fx' THEN 0
              WHEN 'fy' THEN 1
              WHEN 'cx' THEN 2
              WHEN 'cy' THEN 3
            END;
        """)
        rows = cur.fetchall()
        conn.close()
        return [value for (_, value) in rows]

    def fetch_realsense_sim_params(self):
        conn = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True, timeout=5.0)
        cur = conn.cursor()
        cur.execute("""
            SELECT name, value
              FROM realsense_tf_sim
            ORDER BY CASE name
              WHEN 'x'     THEN 0
              WHEN 'y'     THEN 1
              WHEN 'z'     THEN 2
              WHEN 'roll'  THEN 3
              WHEN 'pitch' THEN 4
              WHEN 'yaw'   THEN 5
            END;
        """)
        rows = cur.fetchall()
        conn.close()
        return [value for (_, value) in rows]

    def fetch_realsense_real_params(self):
        conn = sqlite3.connect(f'file:{self.db_path}?mode=ro', uri=True, timeout=5.0)
        cur = conn.cursor()
        cur.execute("""
            SELECT name, value
              FROM realsense_tf_real
            ORDER BY CASE name
              WHEN 'x'     THEN 0
              WHEN 'y'     THEN 1
              WHEN 'z'     THEN 2
              WHEN 'roll'  THEN 3
              WHEN 'pitch' THEN 4
              WHEN 'yaw'   THEN 5
            END;
        """)
        rows = cur.fetchall()
        conn.close()
        return [value for (_, value) in rows]
