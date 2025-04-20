BEGIN TRANSACTION;

CREATE TABLE IF NOT EXISTS camera_params_sim (
  name   TEXT    PRIMARY KEY,
  value  REAL    NOT NULL
);
CREATE TABLE IF NOT EXISTS camera_params_real (
  name   TEXT    PRIMARY KEY,
  value  REAL    NOT NULL
);

COMMIT;
