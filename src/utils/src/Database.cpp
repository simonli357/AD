#include <Database.hpp>
#include <ros/package.h>
#include <stdexcept>

Database::Database() : db(nullptr, sqlite3_close) {
	std::string pkg = ros::package::getPath("planning");
	std::string path = pkg + "/share/database.db";

	sqlite3 *raw = nullptr;
	int rc = sqlite3_open(path.c_str(), &raw);
	if (rc != SQLITE_OK) {
		if (raw)
			sqlite3_close(raw);
		throw std::runtime_error("Failed to open SQLite DB");
	}

	db.reset(raw);
}
