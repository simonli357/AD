#pragma once

#include "Action.hpp"
#include <cstdint>
#include <memory>
#include <queue>
#include <unordered_map>
#include <variant>

using ValueType = std::variant<bool, double, int32_t, char>;

class HTN {
  public:
	HTN();
	HTN(HTN &&) = default;
	HTN(const HTN &) = default;
	HTN &operator=(HTN &&) = default;
	HTN &operator=(const HTN &) = default;
	~HTN();

	std::unordered_map<PRIMITIVES, ValueType> initial_state;
	std::unordered_map<PRIMITIVES, ValueType> goal_state;

	std::queue<std::unique_ptr<Action>> plan;

  private:
};
