#include "htn/actions/Park.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

Park::Park(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
    cost = 0; // Parking is mandatory
	pre_conditions = {
        {FORCE_STOP, false},
		{PARKING_SIGN_DETECTED, true},
        {PARKING_COUNT, 0},
	};
}

Park::~Park() {}

void Park::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Park the car
    utils.debug("Performing Action: Park.", 2);
    // TODO: Park the car
	update_post_conditions();
}

void Park::update_post_conditions() {
    current_state[PARKING_SIGN_DETECTED] = false;
    // Set to 0 if cannot park (occupied slot ...).
    current_state[PARKING_COUNT] = 1;
}
