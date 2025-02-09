#include "htn/actions/ObstacleStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

ObstacleStop::ObstacleStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
    cost = 3; // Navigating around could save some time, stop if we must absolutely stop.
	pre_conditions = {
		{PARKING_SIGN_DETECTED, '_'}, {PARKING_COUNT, '_'}, {TRAFFIC_LIGHT_DETECTED, '_'}, {STOP_SIGN_DETECTED, '_'}, {OBSTACLE_DETECTED, true}, {DESTINATION_REACHED, '_'},
	};
}

void ObstacleStop::execute() {
	if (!can_execute()) {
		std::cout << "Illegal action, pre conditions not satisfied" << std::endl;
		return;
	}
    // TODO: Stop the car, go backwards ...
	update_post_conditions();
}

void ObstacleStop::update_post_conditions() {
    post_conditions[OBSTACLE_DETECTED] = false;
}
