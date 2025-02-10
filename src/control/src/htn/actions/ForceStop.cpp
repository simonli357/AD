#include "htn/actions/ForceStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

ForceStop::ForceStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
    cost = 0;
	pre_conditions = {
        {FORCE_STOP, true},
		{PARKING_SIGN_DETECTED, '_'},
        {PARKING_COUNT, '_'},
        {TRAFFIC_LIGHT_DETECTED, '_'},
        {STOP_SIGN_DETECTED, true},
        {OBSTACLE_DETECTED, '_'},
        {DESTINATION_REACHED, '_'},
	};
}

void ForceStop::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Stop the car until we can restart it
    utils.debug("Performing Action: Force Stop.", 2);
    while (true) {
        auto& condition = world.current_state[FORCE_STOP];
        if (auto* value = std::get_if<bool>(&condition)) {
            if (!(*value)) {
                break;
            }
        }
        utils.publish_cmd_vel(0.0, 0.0);
        world.rate->sleep();
    }
	update_post_conditions();
}

void ForceStop::update_post_conditions() {
    current_state[FORCE_STOP] = false;
}
