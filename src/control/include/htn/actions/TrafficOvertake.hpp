#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class TrafficOvertake : public Action {
  public:
	TrafficOvertake(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	TrafficOvertake(TrafficOvertake &&) = delete;
	TrafficOvertake(const TrafficOvertake &) = delete;
	TrafficOvertake &operator=(TrafficOvertake &&) = delete;
	TrafficOvertake &operator=(const TrafficOvertake &) = delete;
	~TrafficOvertake();

	void execute() override;
};
