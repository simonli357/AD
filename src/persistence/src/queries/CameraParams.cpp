#include "queries/CameraParams.hpp"
#include "sqlite3.h"
#include <iostream>
#include <stdexcept>

CameraParams::CameraParams(Database &db) : db(db) {}

std::array<double, 4> CameraParams::fetch_camera_sim_params() {
	const char *sql = R"(
      SELECT name, value
      FROM camera_params_sim
      ORDER BY CASE name
        WHEN 'fx' THEN 0
        WHEN 'fy' THEN 1
        WHEN 'cx' THEN 2
        WHEN 'cy' THEN 3
      END;
    )";

	sqlite3_stmt *stmt = nullptr;
	if (sqlite3_prepare_v2(db.conn.get(), sql, -1, &stmt, nullptr) != SQLITE_OK) {
		throw std::runtime_error("Failed to prepare table");
    }

	std::array<double, 4> out;
	int idx = 0;
	while (sqlite3_step(stmt) == SQLITE_ROW) {
		const char *name = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
		double val = sqlite3_column_double(stmt, 1);
        std::cout << "CAM: "  << val << std::endl;
		out[idx++] = val;
	}
	sqlite3_finalize(stmt);
	return out;
}

std::array<double, 4> CameraParams::fetch_camera_real_params() {
	const char *sql = R"(
      SELECT name, value
      FROM camera_params_real
      ORDER BY CASE name
        WHEN 'fx' THEN 0
        WHEN 'fy' THEN 1
        WHEN 'cx' THEN 2
        WHEN 'cy' THEN 3
      END;
    )";

	sqlite3_stmt *stmt = nullptr;
	if (sqlite3_prepare_v2(db.conn.get(), sql, -1, &stmt, nullptr) != SQLITE_OK) {
		throw std::runtime_error("Failed to prepare table");
    }

	std::array<double, 4> out;
	int idx = 0;
	while (sqlite3_step(stmt) == SQLITE_ROW) {
		const char *name = reinterpret_cast<const char *>(sqlite3_column_text(stmt, 0));
		double val = sqlite3_column_double(stmt, 1);
		out[idx++] = val;
	}
	sqlite3_finalize(stmt);
	return out;
}
