#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class HandleObstacle : public Action {
  public:
	HandleObstacle(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	HandleObstacle(HandleObstacle &&) = delete;
	HandleObstacle(const HandleObstacle &) = delete;
	HandleObstacle &operator=(HandleObstacle &&) = delete;
	HandleObstacle &operator=(const HandleObstacle &) = delete;
	~HandleObstacle();

	void execute() override;
};
