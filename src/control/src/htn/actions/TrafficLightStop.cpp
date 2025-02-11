#include "htn/actions/TrafficLightStop.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

TrafficLightStop::TrafficLightStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 2;
	pre_conditions = {
        {FORCE_STOP, false},
        {TRAFFIC_LIGHT_DETECTED, true},
	};
}

TrafficLightStop::~TrafficLightStop() {}

void TrafficLightStop::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Stop car
    utils.debug("Performing Action: Traffic Light Stop.", 2);
    // TODO: Implement
}
