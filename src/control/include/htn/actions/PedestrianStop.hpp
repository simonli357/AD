#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class PedestrianStop : public Action {
  public:
	PedestrianStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	PedestrianStop(PedestrianStop &&) = delete;
	PedestrianStop(const PedestrianStop &) = delete;
	PedestrianStop &operator=(PedestrianStop &&) = delete;
	PedestrianStop &operator=(const PedestrianStop &) = delete;
	~PedestrianStop();

	void execute() override;
};
