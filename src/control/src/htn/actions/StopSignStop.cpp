#include "htn/actions/StopSignStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

StopSignStop::StopSignStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
	pre_conditions = {
		{PARKING_SIGN_DETECTED, '_'}, {PARKING_COUNT, '_'}, {TRAFFIC_LIGHT_DETECTED, '_'}, {STOP_SIGN_DETECTED, true}, {OBSTACLE_DETECTED, '_'}, {DESTINATION_REACHED, '_'},
	};
}

void StopSignStop::execute() {
	if (!can_execute()) {
		std::cout << "Illegal action, pre conditions not satisfied" << std::endl;
		return;
	}
    // TODO: Stop the car
	update_post_conditions();
}

void StopSignStop::update_post_conditions() {
    post_conditions[STOP_SIGN_DETECTED] = false;
}
