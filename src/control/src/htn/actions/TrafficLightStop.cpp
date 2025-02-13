#include "htn/actions/TrafficLightStop.hpp"
#include "htn/Action.hpp"
#include "std_msgs/Byte.h"
#include <unordered_map>

TrafficLightStop::TrafficLightStop(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
	cost = 0;
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
    wait_for_green();
    current_state[TRAFFIC_LIGHT_DETECTED] = false;
}

void TrafficLightStop::wait_for_green() {
	if (world.has_light) {
		int neareastDirection = Utility::nearest_direction_index(utils.get_yaw());
		static std::string light_topic_name;
		if (neareastDirection == 0)
			light_topic_name = "/east_traffic_light";
		else if (neareastDirection == 1)
			light_topic_name = "/north_traffic_light";
		else if (neareastDirection == 2)
			light_topic_name = "/west_traffic_light";
		else if (neareastDirection == 3)
			light_topic_name = "/south_traffic_light";
		auto is_green = ros::topic::waitForMessage<std_msgs::Byte>(light_topic_name, ros::Duration(3));
		int n = 0;
		double ekf_x, ekf_y;
		double total_x, total_y;
		while (is_green->data != 1) {
			is_green = ros::topic::waitForMessage<std_msgs::Byte>(light_topic_name, ros::Duration(3));
			utils.publish_cmd_vel(0, 0);
			if (utils.useEkf) {
				utils.get_ekf_states(ekf_x, ekf_y);
				total_x += ekf_x;
				total_y += ekf_y;
				n++;
			}
			world.rate->sleep();
		}
		utils.debug("wait_for_green(): light turned green, proceeding...", 2);
		if (utils.useEkf) {
			utils.x0 = total_x / n - utils.odomX;
			utils.y0 = total_y / n - utils.odomY;
		}
		return;
	} else if (true) {
		utils.debug("wait_for_green(): red light detected, waiting for " + std::to_string(world.stop_duration * 2) + "s or until light turns green", 2);
		auto expiring_time = ros::Time::now() + ros::Duration(5.0);
        int green_count = 0;
        while (true) {
            if (ros::Time::now() > expiring_time) {
                utils.debug("wait_for_green(): timer expired, proceeding...", 2);
                return;
            }
            int sign_index = utils.object_index(OBJECT::GREENLIGHT);
            if (sign_index >= 0) {
                utils.debug("wait_for_green(): green light detected, proceeding...", 2);
                green_count++;
                if (green_count > 5) return;
            }
            if (sign_index < 0) sign_index = utils.object_index(OBJECT::YELLOWLIGHT);
            if (sign_index >= 0) {
                utils.debug("wait_for_green(): yellow light detected, proceeding...", 2);
                green_count++;
                if (green_count > 5) return;
            }
            stop_car_for(world.T);
        }
		return;
	} else {
		utils.debug("wait_for_green(): light detected, but in simulation, proceeding ", 2);
		// stop_for(stop_duration);
	}
}
