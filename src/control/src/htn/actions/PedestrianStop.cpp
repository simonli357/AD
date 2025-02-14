#include "htn/actions/PedestrianStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

PedestrianStop::PedestrianStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 0;
	pre_conditions = {
        {FORCE_STOP, false},
        {PEDESTRIAN_DETECTED, true},
	};
}

PedestrianStop::~PedestrianStop() {}

void PedestrianStop::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    utils.debug("Performing Action: Pedestrian Stop.", 2);
    while (true) {
        object_detection.detect_objects();
        auto& condition = current_state[PEDESTRIAN_DETECTED];
        if (auto* value = std::get_if<bool>(&condition)) {
            if (!(*value)) {
                break;
            }
        }
        utils.publish_cmd_vel(0.0, 0.0);
        world.rate->sleep();
    }
}
