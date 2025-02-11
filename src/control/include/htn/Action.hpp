#pragma once

#include "PathManager.hpp"
#include "World.hpp"
#include "htn/ValueTypes.hpp"
#include "utility.hpp"
#include <cstdint>
#include <unordered_map>

class Action {
  public:
	Action(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	Action(Action &&) = delete;
	Action(const Action &) = delete;
	Action &operator=(Action &&) = delete;
	Action &operator=(const Action &) = delete;
	~Action();

	World &world;
	Utility &utils;
	PathManager &path_manager;
	Eigen::Vector3d &x_current;

	int32_t cost;
	std::unordered_map<PRIMITIVES, ValueType> pre_conditions;
	std::unordered_map<PRIMITIVES, ValueType> &current_state;

	bool can_execute();
	void stop_car_for(double duration);

	virtual void execute();
};
