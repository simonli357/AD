#pragma once

#include "Action.hpp"
#include <cstdint>
#include <memory>
#include <unordered_map>
#include <variant>

using ValueType = std::variant<bool, double, int32_t, char>;

class HTN {
  public:
	HTN(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state, std::unordered_map<PRIMITIVES, ValueType> &goal_state, std::vector<std::unique_ptr<Action>> &actions);
	HTN(HTN &&) = default;
	HTN(const HTN &) = default;
	HTN &operator=(HTN &&) = delete;
	HTN &operator=(const HTN &) = delete;
	~HTN();

	World &world;

	void start();

  private:
	std::unordered_map<PRIMITIVES, ValueType> &current_state;
	std::unordered_map<PRIMITIVES, ValueType> &goal_state;
	std::vector<std::unique_ptr<Action>> &actions;
	void sort_actions();
	bool goal_reached();
};
