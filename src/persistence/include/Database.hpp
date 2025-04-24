#pragma once

#include <memory>
#include <sqlite3.h>
#include <string>

class CameraQueries;
class GraphQueries;

class Database {
  public:
	Database();
	Database(Database &&) = default;
	Database(const Database &) = delete;
	Database &operator=(Database &&) = delete;
	Database &operator=(const Database &) = delete;
	~Database();

	using DB = std::unique_ptr<sqlite3, decltype(&sqlite3_close)>;
	std::string pkg_path;
	DB conn;

	std::unique_ptr<CameraQueries> cam_queries;
	std::unique_ptr<GraphQueries> graph_queries;

	void check_rc(int rc, sqlite3 *db, const char *msg);
	void print_tables();

  private:
	void initialize_tables();
};
