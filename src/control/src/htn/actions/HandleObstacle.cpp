#include "htn/actions/HandleObstacle.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

HandleObstacle::HandleObstacle(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 3; // Navigating around could save some time, stop if we must absolutely stop.
	pre_conditions = {
        {FORCE_STOP, false},
        {OBSTACLE_DETECTED, true},
	};
}

HandleObstacle::~HandleObstacle() {}

void HandleObstacle::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    utils.debug("Performing Action: Handle Obstacle.", 2);
    // TODO: Implement
}
