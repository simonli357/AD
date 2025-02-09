#include "htn/HTN.hpp"
#include "htn/Action.hpp"

HTN::HTN() {
	initial_state = {{PARKING_SIGN_DETECTED, '_'}, {PARKING_COUNT, 0}, {TRAFFIC_LIGHT_DETECTED, '_'}, {STOP_SIGN_DETECTED, '_'}, {OBSTACLE_DETECTED, '_'}, {DESTINATION_REACHED, false}};
	goal_state = {{PARKING_SIGN_DETECTED, '_'}, {PARKING_COUNT, '_'}, {TRAFFIC_LIGHT_DETECTED, '_'}, {STOP_SIGN_DETECTED, '_'}, {OBSTACLE_DETECTED, '_'}, {DESTINATION_REACHED, true}};
}

void HTN::start() {
    std::vector<Action> actions;
}
