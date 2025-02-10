#include "htn/actions/StopSignStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

StopSignStop::StopSignStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
    cost = 2;
	pre_conditions = {
        {FORCE_STOP, false},
		{PARKING_SIGN_DETECTED, '_'},
        {PARKING_COUNT, '_'},
        {TRAFFIC_LIGHT_DETECTED, '_'},
        {STOP_SIGN_DETECTED, true},
        {OBSTACLE_DETECTED, '_'},
        {DESTINATION_REACHED, '_'},
	};
}

StopSignStop::~StopSignStop() {}

void StopSignStop::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Stop the car
    utils.debug("Performing Action: Stop Sign Stop.", 2);
    // TODO: Stop the car
	update_post_conditions();
}

void StopSignStop::update_post_conditions() {
    current_state[STOP_SIGN_DETECTED] = false;
}
