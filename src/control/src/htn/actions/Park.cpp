#include "htn/actions/Park.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

Park::Park(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
    cost = 0; // Parking is mandatory
	pre_conditions = {
        {FORCE_STOP, false},
		{PARKING_SIGN_DETECTED, true},
        {PARKING_COUNT, 0},
        {TRAFFIC_LIGHT_DETECTED, '_'},
        {STOP_SIGN_DETECTED, '_'},
        {OBSTACLE_DETECTED, '_'},
        {DESTINATION_REACHED, '_'},
	};
}

void Park::execute() {
	if (!can_execute()) {
		std::cout << "Illegal action, pre conditions not satisfied" << std::endl;
		return;
	}
    // TODO: Park the car
	update_post_conditions();
}

void Park::update_post_conditions() {
    current_state[PARKING_SIGN_DETECTED] = false;
    // Set to 0 if cannot park (occupied slot ...).
    current_state[PARKING_COUNT] = 1;
}
