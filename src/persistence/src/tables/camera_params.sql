BEGIN TRANSACTION;

CREATE TABLE IF NOT EXISTS camera_params_sim (
  name TEXT PRIMARY KEY,
  value REAL NOT NULL
);

CREATE TABLE IF NOT EXISTS camera_params_real (
  name TEXT PRIMARY KEY,
  value REAL NOT NULL
);

INSERT OR REPLACE INTO camera_params_sim (name, value) VALUES
  ('fx', 554.3826904296875),
  ('fy', 554.3826904296875),
  ('cx', 320.0),
  ('cy', 240.0);

INSERT OR REPLACE INTO camera_params_real (name, value) VALUES
  ('fx', 607.40564),
  ('fy', 607.05829),
  ('cx', 322.97223),
  ('cy', 244.39398);

COMMIT;
