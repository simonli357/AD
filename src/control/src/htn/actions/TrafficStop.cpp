#include "htn/actions/TrafficStop.hpp"
#include "World.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

TrafficStop::TrafficStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
	cost = 2;
	pre_conditions = {
		{FORCE_STOP, false},
		{CAR_DETECTED_ON_SAME_LANE, true},
        {LANE_IS_DOTTED, false}
	};
}

TrafficStop::~TrafficStop() {}

void TrafficStop::execute() {
	if (!can_execute()) {
		utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
	// HandleTraffic the car
	utils.debug("Performing Action: Traffic Stop.", 2);
    stop_car_for(20 * world.T);
    current_state[CAR_DETECTED_ON_SAME_LANE] = false;
}
