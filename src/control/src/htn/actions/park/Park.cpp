#include "htn/actions/park/Park.hpp"
#include "ForceStop.hpp"
#include "HTN.hpp"
#include "park/Parking.hpp"
#include "PedestrianStop.hpp"
#include "World.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

Park::Park(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
	cost = 1; // Parking is mandatory
	pre_conditions = {
		{FORCE_STOP, false},
		{PARKING_SIGN_DETECTED, true},
		{PARKING_COUNT, 0},
	};
}

Park::~Park() {}

void Park::htn() {
    std::unordered_map<PRIMITIVES, ValueType> initial_state = {
        {FORCE_STOP, false},
        {PEDESTRIAN_DETECTED, false},
        {PARKING_COMPLETE , false},
        {PARKING_SUCCESS, false}
    };
    std::unordered_map<PRIMITIVES, ValueType> goal_state = {
        {PARKING_COMPLETE , true}
    };

    std::vector<std::unique_ptr<Action>> actions;
    actions.push_back(std::make_unique<ForceStop>(world, initial_state));
    actions.push_back(std::make_unique<PedestrianStop>(world, initial_state));
    actions.push_back(std::make_unique<Parking>(world, initial_state));

    HTN(world, initial_state, goal_state, actions).start();

    auto& condition = initial_state[PARKING_SUCCESS];
    if (auto* value = std::get_if<bool>(&condition)) {
        if (*value) {
            current_state[PARKING_COUNT] = 1;
        }
    }
}

void Park::execute() {
	if (!can_execute()) {
		utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
	// Park the car
	utils.debug("Performing Action: Park.", 2);
    htn();
    current_state[PARKING_SIGN_DETECTED] = false;
}
