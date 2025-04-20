#pragma once

#include <memory>
#include <sqlite3.h>
#include <string>

class CameraParams;

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

    std::unique_ptr<CameraParams> cam_queries;

	void print_tables();

  private:
	void initialize_tables();
};
