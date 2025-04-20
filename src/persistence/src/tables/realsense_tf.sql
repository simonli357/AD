BEGIN TRANSACTION;

CREATE TABLE IF NOT EXISTS realsense_tf_sim (
  name   TEXT    PRIMARY KEY,
  value  REAL    NOT NULL
);
CREATE TABLE IF NOT EXISTS realsense_tf_real (
  name   TEXT    PRIMARY KEY,
  value  REAL    NOT NULL
);

COMMIT;
