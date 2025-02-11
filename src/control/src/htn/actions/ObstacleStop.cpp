#include "htn/actions/ObstacleStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

ObstacleStop::ObstacleStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 3; // Navigating around could save some time, stop if we must absolutely stop.
	pre_conditions = {
        {FORCE_STOP, false},
        {OBSTACLE_DETECTED, true},
	};
}

ObstacleStop::~ObstacleStop() {}

void ObstacleStop::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Stop car
    utils.debug("Performing Action: Obstacle Stop.", 2);
    // TODO: Stop the car, go backwards ...
	update_post_conditions();
}

void ObstacleStop::update_post_conditions() {
    current_state[OBSTACLE_DETECTED] = false;
}
