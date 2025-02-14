#include "htn/Action.hpp"
#include "ObjectDetection.hpp"
#include <unordered_map>

Action::Action(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : world(world), utils(world.utils), path_manager(world.path_manager), x_current(world.x_current), current_state(current_state), object_detection(world, current_state) {}

Action::~Action() {}

bool Action::can_execute() {
    object_detection.detect_objects();
	for (const auto &[key, value] : pre_conditions) {
		if (pre_conditions[key] != current_state[key]) {
			return false;
		}
	}
	return true;
}

//-----------------//
// Common methods  //
// ----------------//

void Action::stop_car_for(double duration) {
    ros::Time timer = ros::Time::now() + ros::Duration(duration);
    while (ros::Time::now() < timer) {
        utils.publish_cmd_vel(0.0, 0.0);
        world.rate->sleep();
    }
    // TODO: reset path manager idx
}


// Interrupts current action by stopping the car until it can be continued.
void Action::emergency_stop() {
    while(!can_execute()) {
        utils.publish_cmd_vel(0.0, 0.0);
        world.rate->sleep();
    }
}

//-----------------//
// Virtual methods //
// ----------------//

void Action::execute() { return; }
