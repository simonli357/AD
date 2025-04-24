#include "queries/CameraQueries.hpp"
#include "sqlite3.h"

CameraQueries::CameraQueries(Database &db) : db(db) {}

void CameraQueries::set_camera_sim_params(const std::array<double,4> &params) {
    static const char* names[4] = {"fx","fy","cx","cy"};
    char *err = nullptr;
    db.check_rc(sqlite3_exec(db.conn.get(), "BEGIN;", nullptr, nullptr, &err), db.conn.get(), "BEGIN failed");
    sqlite3_stmt *stmt = nullptr;
    const char *sql = "INSERT OR REPLACE INTO camera_params_sim (name,value) VALUES (?,?);";
    db.check_rc(sqlite3_prepare_v2(db.conn.get(), sql, -1, &stmt, nullptr), db.conn.get(), "prepare failed");
    for (int i = 0; i < 4; ++i) {
        sqlite3_bind_text   (stmt, 1, names[i], -1, SQLITE_STATIC);
        sqlite3_bind_double (stmt, 2, params[i]);
        db.check_rc(sqlite3_step  (stmt), db.conn.get(), "step failed");
        db.check_rc(sqlite3_reset (stmt), db.conn.get(), "reset failed");
    }
    sqlite3_finalize(stmt);
    db.check_rc(sqlite3_exec(db.conn.get(), "COMMIT;", nullptr, nullptr, &err), db.conn.get(), "COMMIT failed");
}

void CameraQueries::set_camera_real_params(const std::array<double,4> &params) {
    static const char* names[4] = {"fx","fy","cx","cy"};
    char *err = nullptr;
    db.check_rc(sqlite3_exec(db.conn.get(), "BEGIN;", nullptr, nullptr, &err), db.conn.get(), "BEGIN failed");
    sqlite3_stmt *stmt = nullptr;
    const char *sql = "INSERT OR REPLACE INTO camera_params_real (name,value) VALUES (?,?);";
    db.check_rc(sqlite3_prepare_v2(db.conn.get(), sql, -1, &stmt, nullptr), db.conn.get(), "prepare failed");
    for (int i = 0; i < 4; ++i) {
        sqlite3_bind_text   (stmt, 1, names[i], -1, SQLITE_STATIC);
        sqlite3_bind_double (stmt, 2, params[i]);
        db.check_rc(sqlite3_step  (stmt), db.conn.get(), "step failed");
        db.check_rc(sqlite3_reset (stmt), db.conn.get(), "reset failed");
    }
    sqlite3_finalize(stmt);
    db.check_rc(sqlite3_exec(db.conn.get(), "COMMIT;", nullptr, nullptr, &err), db.conn.get(), "COMMIT failed");
}

void CameraQueries::set_realsense_tf_sim_params(const std::array<double,6> &params) {
    static const char* names[6] = {"x","y","z","roll","pitch","yaw"};
    char *err = nullptr;
    db.check_rc(sqlite3_exec(db.conn.get(), "BEGIN;", nullptr, nullptr, &err), db.conn.get(), "BEGIN failed");
    sqlite3_stmt *stmt = nullptr;
    const char *sql = "INSERT OR REPLACE INTO realsense_tf_sim (name,value) VALUES (?,?);";
    db.check_rc(sqlite3_prepare_v2(db.conn.get(), sql, -1, &stmt, nullptr), db.conn.get(), "prepare failed");
    for (int i = 0; i < 6; ++i) {
        sqlite3_bind_text   (stmt, 1, names[i], -1, SQLITE_STATIC);
        sqlite3_bind_double (stmt, 2, params[i]);
        db.check_rc(sqlite3_step  (stmt), db.conn.get(), "step failed");
        db.check_rc(sqlite3_reset (stmt), db.conn.get(), "reset failed");
    }
    sqlite3_finalize(stmt);
    db.check_rc(sqlite3_exec(db.conn.get(), "COMMIT;", nullptr, nullptr, &err), db.conn.get(), "COMMIT failed");
}

void CameraQueries::set_realsense_tf_real_params(const std::array<double,6> &params) {
    static const char* names[6] = {"x","y","z","roll","pitch","yaw"};
    char *err = nullptr;
    db.check_rc(sqlite3_exec(db.conn.get(), "BEGIN;", nullptr, nullptr, &err), db.conn.get(), "BEGIN failed");
    sqlite3_stmt *stmt = nullptr;
    const char *sql = "INSERT OR REPLACE INTO realsense_tf_real (name,value) VALUES (?,?);";
    db.check_rc(sqlite3_prepare_v2(db.conn.get(), sql, -1, &stmt, nullptr), db.conn.get(), "prepare failed");
    for (int i = 0; i < 6; ++i) {
        sqlite3_bind_text   (stmt, 1, names[i], -1, SQLITE_STATIC);
        sqlite3_bind_double (stmt, 2, params[i]);
        db.check_rc(sqlite3_step  (stmt), db.conn.get(), "step failed");
        db.check_rc(sqlite3_reset (stmt), db.conn.get(), "reset failed");
    }
    sqlite3_finalize(stmt);
    db.check_rc(sqlite3_exec(db.conn.get(), "COMMIT;", nullptr, nullptr, &err), db.conn.get(), "COMMIT failed");
}
