#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class TrafficStop : public Action {
  public:
	TrafficStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	TrafficStop(TrafficStop &&) = delete;
	TrafficStop(const TrafficStop &) = delete;
	TrafficStop &operator=(TrafficStop &&) = delete;
	TrafficStop &operator=(const TrafficStop &) = delete;
	~TrafficStop();

	void execute() override;
};
