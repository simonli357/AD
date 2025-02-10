#include "htn/HTN.hpp"
#include "htn/Action.hpp"
#include <memory>

HTN::HTN(std::unordered_map<PRIMITIVES, ValueType> &current_state, std::unordered_map<PRIMITIVES, ValueType> &goal_state, std::vector<std::unique_ptr<Action>> &actions)
	: current_state(current_state), goal_state(goal_state), actions(actions) {
	sort_actions();
}

HTN::~HTN() {}

void HTN::sort_actions() {
	std::sort(actions.begin(), actions.end(), [](const std::unique_ptr<Action> &a, const std::unique_ptr<Action> &b) { return a->cost < b->cost; });
}

bool HTN::goal_reached() {
	for (const auto &[key, value] : goal_state) {
		// wildcard
		if (std::holds_alternative<char>(goal_state[key]) && std::get<char>(goal_state[key]) == '_') {
			continue;
		}
		if (std::holds_alternative<char>(current_state[key]) && std::get<char>(current_state[key]) == '_') {
			continue;
		}
		if (goal_state[key] != current_state[key]) {
			return false;
		}
	}
	return true;
}

void HTN::start() {
	while (true) {
		if (goal_reached()) {
			break;
		}
		for (auto &action : actions) {
			if (action) {
                action->set_conditions(current_state);
                if (action->can_execute()) {
                    action->execute();
                    break;
                }
			}
		}
	}
}
