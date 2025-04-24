#include "queries/GraphQueries.hpp"
#include "sqlite3.h"

GraphQueries::GraphQueries(Database &db) : db(db) {}


void GraphQueries::set_graph(const std::string &graph) {
    static constexpr char const* NAME = "graph";
    char *err = nullptr;

    db.check_rc(
        sqlite3_exec(db.conn.get(), "BEGIN;", nullptr, nullptr, &err),
        db.conn.get(),
        "BEGIN failed"
    );

    sqlite3_stmt *stmt = nullptr;
    static constexpr char const* SQL =
        "INSERT OR REPLACE INTO graph(name,value) VALUES(?,?);";
    db.check_rc(
        sqlite3_prepare_v2(db.conn.get(), SQL, -1, &stmt, nullptr),
        db.conn.get(),
        "sqlite3_prepare_v2 failed"
    );

    db.check_rc(
        sqlite3_bind_text(stmt, 1, NAME, -1, SQLITE_STATIC),
        db.conn.get(),
        "sqlite3_bind_text(name) failed"
    );
    db.check_rc(
        sqlite3_bind_text(stmt, 2, graph.c_str(), -1, SQLITE_TRANSIENT),
        db.conn.get(),
        "sqlite3_bind_text(graph) failed"
    );

    db.check_rc(
        sqlite3_step(stmt),
        db.conn.get(),
        "sqlite3_step failed"
    );

    sqlite3_finalize(stmt);

    db.check_rc(
        sqlite3_exec(db.conn.get(), "COMMIT;", nullptr, nullptr, &err),
        db.conn.get(),
        "COMMIT failed"
    );
}

