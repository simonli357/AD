#include "htn/actions/ForceStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

ForceStop::ForceStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 0;
	pre_conditions = {
        {FORCE_STOP, true},
	};
}

ForceStop::~ForceStop() {}

void ForceStop::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Stop the car until we can restart it
    utils.debug("Performing Action: Force Stop.", 2);
    while (true) {
        auto& condition = current_state[FORCE_STOP];
        if (auto* value = std::get_if<bool>(&condition)) {
            if (!(*value)) {
                break;
            }
        }
        utils.publish_cmd_vel(0.0, 0.0);
        world.rate->sleep();
    }
}
