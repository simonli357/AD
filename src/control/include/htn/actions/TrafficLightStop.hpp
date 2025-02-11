#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class TrafficLightStop : public Action {
  public:
	TrafficLightStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	TrafficLightStop(TrafficLightStop &&) = delete;
	TrafficLightStop(const TrafficLightStop &) = delete;
	TrafficLightStop &operator=(TrafficLightStop &&) = delete;
	TrafficLightStop &operator=(const TrafficLightStop &) = delete;
	~TrafficLightStop();

	void execute() override;
};
