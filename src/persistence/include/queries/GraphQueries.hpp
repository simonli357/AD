#pragma once

#include "Database.hpp"

class GraphQueries {
  public:
	GraphQueries(Database &db);
	GraphQueries(GraphQueries &&) = default;
	GraphQueries(const GraphQueries &) = delete;
	GraphQueries &operator=(GraphQueries &&) = delete;
	GraphQueries &operator=(const GraphQueries &) = delete;
	~GraphQueries() = default;

	Database &db;

	void set_graph(const std::string &graph);
};
