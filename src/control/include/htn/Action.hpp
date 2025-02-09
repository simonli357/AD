#pragma once

#include "PathManager.hpp"
#include "Primitives.hpp"
#include "World.hpp"
#include "utility.hpp"
#include <cstdint>
#include <unordered_map>
#include <variant>

using ValueType = std::variant<bool, double, int32_t, char>;

class Action {
  public:
	Action(World &world, std::unordered_map<PRIMITIVES, ValueType> &contidions);
	Action(Action &&) = default;
	Action(const Action &) = default;
	Action &operator=(Action &&) = delete;
	Action &operator=(const Action &) = delete;
	~Action();

	World &world;
	Utility &utils = world.utils;
	PathManager &path_manager = world.path_manager;
	Eigen::Vector3d &x_current = world.x_current;

	int32_t cost;
	std::unordered_map<PRIMITIVES, ValueType> pre_conditions;
	std::unordered_map<PRIMITIVES, ValueType> post_conditions;

	bool can_execute();
	virtual void execute();

  private:
	virtual void update_post_conditions();
};
