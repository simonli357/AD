#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class StopSignStop : public Action {
  public:
	StopSignStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	StopSignStop(StopSignStop &&) = delete;
	StopSignStop(const StopSignStop &) = delete;
	StopSignStop &operator=(StopSignStop &&) = delete;
	StopSignStop &operator=(const StopSignStop &) = delete;
	~StopSignStop();

	void execute() override;
};
