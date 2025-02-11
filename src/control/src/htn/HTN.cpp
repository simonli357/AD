#include "htn/HTN.hpp"
#include "htn/Action.hpp"
#include "object-detection/ObjectDetection.hpp"
#include <memory>

HTN::HTN(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state, std::unordered_map<PRIMITIVES, ValueType> &goal_state, std::vector<std::unique_ptr<Action>> &actions)
	: world(world), current_state(current_state), goal_state(goal_state), actions(actions) {
	sort_actions();
}

HTN::~HTN() {}

void HTN::sort_actions() {
	std::sort(actions.begin(), actions.end(), [](const std::unique_ptr<Action> &a, const std::unique_ptr<Action> &b) { return a->cost < b->cost; });
}

bool HTN::goal_reached() {
	for (const auto &[key, value] : goal_state) {
		if (goal_state[key] != current_state[key]) {
			return false;
		}
	}
	return true;
}

void HTN::start() {
	while (true) {
        world.utils.update_states(world.x_current);
		if (goal_reached()) {
			break;
		}
        ObjectDetection(world, current_state).detect_objects();
		for (auto &action : actions) {
			if (action) {
                if (action->can_execute()) {
                    action->execute();
                    world.rate->sleep();
                    break;
                }
			}
		}
	}
}
