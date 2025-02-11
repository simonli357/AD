#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class ForceStop : public Action {
  public:
	ForceStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	ForceStop(ForceStop &&) = delete;
	ForceStop(const ForceStop &) = delete;
	ForceStop &operator=(ForceStop &&) = delete;
	ForceStop &operator=(const ForceStop &) = delete;
	~ForceStop();

	void execute() override;
};
