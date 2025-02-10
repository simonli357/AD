#include "htn/actions/TrafficLightStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

TrafficLightStop::TrafficLightStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
    cost = 2;
	pre_conditions = {
        {FORCE_STOP, false},
		{PARKING_SIGN_DETECTED, '_'},
        {PARKING_COUNT, '_'},
        {TRAFFIC_LIGHT_DETECTED, true},
        {STOP_SIGN_DETECTED, '_'},
        {OBSTACLE_DETECTED, '_'},
        {DESTINATION_REACHED, '_'},
	};
}

void TrafficLightStop::execute() {
	if (!can_execute()) {
		std::cout << "Illegal action, pre conditions not satisfied" << std::endl;
		return;
	}
    // TODO: Stop the car
	update_post_conditions();
}

void TrafficLightStop::update_post_conditions() {
    current_state[TRAFFIC_LIGHT_DETECTED] = false;
}
