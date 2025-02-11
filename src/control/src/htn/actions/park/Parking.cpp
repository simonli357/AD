#include "htn/actions/park/Parking.hpp"
#include "ObjectDetection.hpp"
#include "World.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

Parking::Parking(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state) : Action(world, current_state) {
	cost = 0; // Parkinging is mandatory
	pre_conditions = {
		{FORCE_STOP, false},
		{PEDESTRIAN_DETECTED, false},
	};
}

Parking::~Parking() {}

void Parking::execute() {
	if (!can_execute()) {
		utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
	// Parking the car
	utils.debug("Performing Action: Parking.", 2);
	stop_car_for(world.stop_duration / 2);
	double offset_thresh = 0.1;
	double base_offset = world.detected_dist + PARKING_SPOT_LENGTH * 1.5 + offset_thresh;
	double offset = base_offset;
	utils.debug("park sign detected at a distance of: " + std::to_string(world.detected_dist) + ", parking offset is: " + std::to_string(offset), 2);
	// base_offset = 0;
	world.right_park = true;
	bool hard_code = true;
	int target_spot = 0;
	auto temp_rate = ros::Rate(50);
	if (true) {
		double orientation = Utility::nearest_direction(utils.get_yaw());
		ROS_INFO("orientation: %.3f", orientation);
		double x0, y0, yaw0;
		utils.get_states(x0, y0, yaw0);
		int park_index = utils.object_index(OBJECT::PARK);
		if (park_index >= 0) {
			double dist = utils.object_distance(park_index);
		} else {
			ROS_WARN("parking sign invalid... returning to STATE::MOVING");
			current_state[PARKING_COMPLETE] = true;
			return;
		}
		// if (1) {
		if (world.sign_relocalize) {
			auto park_sign_pose = utils.estimate_object_pose2d(x0, y0, yaw0, utils.object_box(park_index), world.detected_dist);
			int success = world.sign_based_relocalization(park_sign_pose, PARKING_SIGN_POSES, "PARKING");
		}
		utils.debug("PARKING(): target spot: " + std::to_string(target_spot), 2);
		for (int i = 0; i < world.PARKING_SPOTS.size(); i++) {
			std::cout << "parking spot " << i << ": " << world.PARKING_SPOTS[i][0] << ", " << world.PARKING_SPOTS[i][1] << std::endl;
		}
		while (1) {
            ObjectDetection(world, current_state).detect_objects();
            if (!can_execute()) {
                return;
            }
			// check utils.recent_car_indices
			// std::cout << "recent car indices size: " << utils.recent_car_indices.size() << std::endl;
			std::list<int> cars = utils.recent_car_indices;
			// std::cout << "number of cars detected: " << cars.size() << std::endl;
			// iterate through all cars and check if any are in the parking spot
			bool changed = false;
			bool car_in_spot = false;
			while (1) {
				for (int i : cars) {
					utils.debug("PARKING(): checking car: (" + std::to_string(utils.detected_cars[i][0]) + ", " + std::to_string(utils.detected_cars[i][1]) +
									"), error: " + std::to_string((utils.detected_cars[i] - world.PARKING_SPOTS[target_spot]).norm()),
								5);
					Eigen::Vector2d world_pose = utils.detected_cars[i];
					Eigen::Vector2d spot = world.PARKING_SPOTS[target_spot];
					double error_sq = (world_pose - spot).squaredNorm();
					double error_threshold_sq = 0.04;
					if (error_sq < error_threshold_sq) {
						car_in_spot = true;
						utils.debug("PARKING(): car detected in spot: " + std::to_string(target_spot) + ", error: " + std::to_string(std::sqrt(error_sq)), 2);
						target_spot++;
						changed = true;
						break;
					}
					car_in_spot = false;
				}
				if (!car_in_spot)
					break;
			}
			if (changed) {
				offset = base_offset + target_spot / 2 * PARKING_SPOT_LENGTH;
				world.right_park = target_spot % 2 == 0;
				ROS_INFO("car in spot, changing to target spot %d at (%.3f, %.3f), right: %s", target_spot, world.PARKING_SPOTS[target_spot][0], world.PARKING_SPOTS[target_spot][1],
						 world.right_park ? "true" : "false");
			}
			double x, y, yaw;
			utils.get_states(x, y, yaw);
			double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
			if (norm_sq >= offset * offset) {
				ROS_INFO("x offset reached: (%.2f, %.2f), ready for parking maneuver...", x, y);
				stop_car_for(world.stop_duration / 2);
				break;
			}
			orientation_follow(orientation);
			// solve();
			// if (norm_sq > offset * offset * 0.2) {
			//     orientation_follow(orientation);
			// } else {
			//     update_mpc_states();
			//     solve();
			// }
			temp_rate.sleep();
		}
	}
	stop_car_for(world.stop_duration / 2);
	if (hard_code) {
		// right_park = true; //temp
		double orientation = Utility::nearest_direction(utils.get_yaw());
		double x, y, yaw;
		utils.get_states(x, y, yaw);
		double initial_y_error = y - (world.PARKING_SPOTS[target_spot][1] + PARKING_SPOT_WIDTH * (world.right_park ? 1 : -1));
		double initial_yaw_error = orientation - yaw;
		initial_yaw_error = Utility::yaw_mod(initial_yaw_error); // normalize to [-pi, pi]
		// ROS_INFO("initial y error: %.3f, initial yaw error: %.3f", initial_y_error, initial_yaw_error);
		utils.debug("orientation: " + std::to_string(orientation) + ", yaw: " + std::to_string(yaw), 4);
		// exit(0);
		parking_maneuver_hardcode(world.right_park, false, 1 / world.T_park, initial_y_error, initial_yaw_error);
	}
	double x, y, yaw;
	utils.get_states(x, y, yaw);
	double x_error = x - world.PARKING_SPOTS[target_spot][0];
	if (std::abs(x_error) > 0.15) {
		double orientation = Utility::nearest_direction(utils.get_yaw());
		ROS_INFO("parked but x offset too large: %.3f, adjusting... orientation: %.3f", x_error, orientation);
		double x0, y0, yaw0;
		utils.get_states(x0, y0, yaw0);
		while (1) {
			x_error = x - world.PARKING_SPOTS[target_spot][0];
			utils.get_states(x, y, yaw);
			double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
			if (x_error > 0 && x_error < 0.05 || x_error < 0 && x_error > -0.05) {
				ROS_INFO("parking spot reached, stopping...");
				utils.publish_cmd_vel(0.0, 0.0);
				break;
			}
			double speed = world.NORMAL_SPEED;
			if (x_error > 0) {
				speed = -speed;
			}
			orientation_follow(orientation, speed);
			temp_rate.sleep();
		}
	}
    current_state[PARKING_SUCCESS] = true;
    current_state[PARKING_COMPLETE] = true;
}

void Parking::orientation_follow(double orientation, double speed) {
	if (speed < -1)
		speed = world.NORMAL_SPEED;
	if (world.pubWaypoints) {
		world.publish_waypoints();
	}
	double yaw_error = orientation - utils.get_yaw();
	if (yaw_error > M_PI * 1.5)
		yaw_error -= 2 * M_PI;
	else if (yaw_error < -M_PI * 1.5)
		yaw_error += 2 * M_PI;
	double steer = -yaw_error * 180 / M_PI * 1;
	if (world.check_crosswalk() > 0) {
		speed *= world.cw_speed_ratio;
	}
	if (world.check_highway() > 0) {
		speed *= world.hw_speed_ratio;
	}
	utils.publish_cmd_vel(steer, speed);
}

int Parking::parking_maneuver_hardcode(bool right, bool exit, double rate_val, double initial_y_error, double initial_yaw_error) {
	// rate_val = 1/mpc.T;
	Eigen::VectorXd targets(3);
	Eigen::VectorXd steerings(3);
	Eigen::VectorXd speeds(3);
	Eigen::VectorXd thresholds(3);
	double base_yaw_target = world.parking_base_yaw_target * M_PI;
	utils.debug("parking_maneuver_hardcode(): base yaw target: " + std::to_string(base_yaw_target / M_PI) + "pi", 3);
	base_yaw_target = base_yaw_target + 0.02 / (0.29 * M_PI) * base_yaw_target * initial_y_error / MAX_PARKING_Y_ERROR * (right ? 1 : -1);
	utils.debug("parking_maneuver_hardcode(): initial y error: " + std::to_string(initial_y_error) + ", initial yaw error: " + std::to_string(initial_yaw_error) +
					", base yaw target: " + std::to_string(base_yaw_target / M_PI) + "pi",
				2);
	double base_steer = -HARD_MAX_STEERING;
	double base_speed = world.parking_base_speed;
	double base_thresh = world.parking_base_thresh;
	targets << base_yaw_target, 0.0, 0.0;
	steerings << -base_steer, base_steer, -base_steer;
	speeds << base_speed, base_speed, -base_speed;
	thresholds << base_thresh, base_thresh, base_thresh / 3;
	if (exit) {
		base_yaw_target *= 0.95;
		targets << base_yaw_target, base_yaw_target, 0.0;
		thresholds(0) = (1 - base_thresh / base_yaw_target) * base_yaw_target;
		thresholds(1) = base_thresh;
		thresholds(2) = base_thresh;
		speeds << base_speed, -base_speed, -base_speed;
	}
	if (!right) {
		steerings *= -1;
		targets *= -1;
		std::cout << "targets: " << targets.transpose() << ", steerings: " << steerings.transpose() << ", speeds: " << speeds.transpose() << ", thresholds: " << thresholds.transpose() << std::endl;
	}
	utils.debug(std::string("parking_maneuver_hardcode(): park right: ") + (right ? "true" : "false") + ", exit: " + std::to_string(exit), 2);
	std::cout << "targets: " << targets.transpose() << ", steerings: " << steerings.transpose() << ", speeds: " << speeds.transpose() << ", thresholds: " << thresholds.transpose() << std::endl;
	return maneuver_hardcode(targets, steerings, speeds, thresholds, rate_val);
}

int Parking::maneuver_hardcode(const Eigen::VectorXd &targets, const Eigen::VectorXd &steerings, const Eigen::VectorXd &speeds, const Eigen::VectorXd &thresholds, double rate_val) {
	/*
	targets: target yaws or distances
	thresholds: thresholds for each target
	*/
	double x0, y0, yaw0;
	// get current states
	utils.get_states(x0, y0, yaw0);
	// get closest direction
	yaw0 = Utility::nearest_direction(yaw0);
	yaw0 = Utility::yaw_mod(yaw0);
	Eigen::VectorXd yaw0_vec = Eigen::VectorXd::Constant(targets.size(), yaw0);
	Eigen::VectorXd target_yaws = targets + yaw0_vec;
	// std::cout << "target yaws: " << target_yaws.transpose() << std::endl;
	utils.debug("maneuver_hardcode(): initial yaw: " + std::to_string(yaw0) + ", target yaws: " + std::to_string(target_yaws(0)) + ", " + std::to_string(target_yaws(1)), 2);
	utils.debug("maneuver_hardcode(): nearest direction: " + std::to_string(yaw0), 2);
	int stage = 1;
	int num_stages = targets.size();
	ros::Rate temp_rate(rate_val);

	double steering_angle = steerings(stage - 1);
	double speed = speeds(stage - 1);
	double yaw = utils.get_yaw();
	double yaw_error = yaw - target_yaws(stage - 1);
	double yaw_error_sign = yaw_error > 0 ? 1 : -1;
	while (1) {
		yaw = utils.get_yaw();
		yaw = Utility::yaw_mod(yaw);
		yaw_error = yaw - target_yaws(stage - 1);
		utils.debug("maneuver_hardcode(): stage " + std::to_string(stage) + ", yaw: " + std::to_string(yaw) + ", target yaw: " + std::to_string(target_yaws(stage - 1)) +
						", yaw error: " + std::to_string(yaw_error),
					5);
		while (std::abs(yaw_error) > M_PI * 1.2) {
			if (yaw_error > M_PI * 1.2) {
				yaw_error -= 2 * M_PI;
			} else {
				yaw_error += 2 * M_PI;
			}
		}
		bool exit_cond;
		if (std::abs(steering_angle) < 0.1) {
			double x, y, yaw;
			utils.get_states(x, y, yaw);
			double dist_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
			exit_cond = std::abs(dist_sq - targets(stage - 1) * targets(stage - 1)) < thresholds(stage - 1) * thresholds(stage - 1);
			orientation_follow(yaw0, speed);
		} else {
			if (yaw_error_sign < 0) {
				exit_cond = yaw_error > -thresholds(stage - 1);
			} else {
				exit_cond = yaw_error < thresholds(stage - 1);
			}
			utils.debug("maneuver_hardcode(): yaw error: " + std::to_string(yaw_error) + ", exit condition: " + std::to_string(exit_cond), 5);
			utils.publish_cmd_vel(steering_angle, speed);
		}
		if (exit_cond) {
			utils.debug("stage " + std::to_string(stage) + " completed. yaw error: " + std::to_string(yaw_error), 3);
			stage++;
			if (stage > num_stages) {
				utils.debug("maneuver completed", 2);
				break;
			}
			steering_angle = steerings(stage - 1);
			speed = speeds(stage - 1);
			utils.debug("new stage: " + std::to_string(stage) + ", steer: " + std::to_string(steering_angle) + ", speed: " + std::to_string(speed), 3);
			yaw_error = yaw - target_yaws(stage - 1);
			yaw_error_sign = yaw_error > 0 ? 1 : -1;
			continue;
		}
		temp_rate.sleep();
	}
	return 0;
}
