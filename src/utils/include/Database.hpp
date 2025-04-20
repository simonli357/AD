#include <memory>
#include <sqlite3.h>

class Database {
  public:
	Database();
	Database(Database &&) = default;
	Database(const Database &) = delete;
	Database &operator=(Database &&) = delete;
	Database &operator=(const Database &) = delete;
	~Database() = default;

	using DB = std::unique_ptr<sqlite3, decltype(&sqlite3_close)>;

	DB db;
};
