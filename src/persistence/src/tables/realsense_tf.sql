BEGIN TRANSACTION;

CREATE TABLE IF NOT EXISTS realsense_tf_sim (
  name   TEXT    PRIMARY KEY,
  value  REAL    NOT NULL
);
CREATE TABLE IF NOT EXISTS realsense_tf_real (
  name   TEXT    PRIMARY KEY,
  value  REAL    NOT NULL
);

DELETE FROM realsense_tf_sim;
DELETE FROM realsense_tf_real;

-- static constexpr std::array<double, 6> REALSENSE_TF = {-0.1, 0.05, 0.2, 0, 0.1, 0};

INSERT OR REPLACE INTO realsense_tf_sim (name, value) VALUES
  ('x',     -0.11),
  ('y',     -0.032),
  ('z',      0.25),
  ('roll',   0.0),
  ('pitch',  0.0),
  ('yaw',    0.0);

INSERT OR REPLACE INTO realsense_tf_real (name, value) VALUES
  ('x',     -0.11),
  ('y',     -0.032),
  ('z',      0.25),
  ('roll',   0.0),
  ('pitch',  0.0),
  ('yaw',    0.0);

COMMIT;
