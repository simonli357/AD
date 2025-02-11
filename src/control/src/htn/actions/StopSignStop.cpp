#include "htn/actions/StopSignStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

StopSignStop::StopSignStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 2;
	pre_conditions = {
        {FORCE_STOP, false},
        {STOP_SIGN_DETECTED, true},
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
