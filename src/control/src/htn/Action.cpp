#include "htn/Action.hpp"
#include <unordered_map>

Action::Action(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : world(world), post_conditions(conditions) {}

bool Action::can_execute() {
	for (const auto &[key, value] : post_conditions) {
		// wildcard
		if (std::holds_alternative<char>(pre_conditions[key]) && std::get<char>(pre_conditions[key]) == '_') {
			continue;
		}
		if (pre_conditions[key] != post_conditions[key]) {
			return false;
		}
	}
	return true;
}

void Action::execute() { return; }
void Action::update_post_conditions() { return; }
