#include "htn/actions/MoveForward.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

MoveForward::MoveForward(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
    cost = 1; // Moving forward is the best move as it moves us closer to the goal.
	pre_conditions = {
		{PARKING_SIGN_DETECTED, '_'}, {PARKING_COUNT, '_'}, {TRAFFIC_LIGHT_DETECTED, false}, {STOP_SIGN_DETECTED, false}, {OBSTACLE_DETECTED, false}, {DESTINATION_REACHED, false},
	};
}

void MoveForward::execute() {
	if (!can_execute()) {
		std::cout << "Illegal action, pre conditions not satisfied" << std::endl;
		return;
	}
	world.update_mpc_states();
	world.solve();
	// TODO : Lane based relocalization
	// TODO : Sign based relocalization
	update_post_conditions();
}

bool MoveForward::detect_stop_sign() {
	int sign_index = utils.object_index(OBJECT::STOPSIGN);
	if (sign_index >= 0) {
		double dist = utils.object_distance(sign_index);
		if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
			// if(lane) utils.reset_odom();
			if (world.sign_in_path(sign_index, dist + 0.2)) {
				utils.debug("Stop sign detected at a distance of: " + std::to_string(dist), 2);
				world.mpc.reset_solver();
				return true;
			}
		}
	}
	return false;
}

bool MoveForward::detect_traffic_light() {
	bool is_red = false;
	int sign_index = utils.object_index(OBJECT::REDLIGHT);
	if (sign_index >= 0) {
		is_red = true;
	} else {
		sign_index = utils.object_index(OBJECT::LIGHTS);
	}
	if (sign_index < 0)
		sign_index = utils.object_index(OBJECT::GREENLIGHT);
	if (sign_index < 0)
		sign_index = utils.object_index(OBJECT::YELLOWLIGHT);
	if (sign_index >= 0) {
		double dist = utils.object_distance(sign_index);
		if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
			// if(lane) utils.reset_odom();
			if (world.sign_in_path(sign_index, dist + 0.2)) {
				utils.debug("Traffic light detected at a distance of: " + std::to_string(dist), 2);
			}
		}
	}
	if (is_red) {
		world.mpc.reset_solver();
		return true;
	}
	return false;
}

bool MoveForward::detect_parking_sign() {
	int park_index = utils.object_index(OBJECT::PARK);
	if (park_index >= 0) {
		double dist = utils.object_distance(park_index);
		if (dist < MAX_PARK_DIST && dist > 0) {
			utils.debug("Parking sign detected at a distance of: " + std::to_string(dist), 2);
			auto x1 = PARKING_SIGN_POSES[0][0];
			auto y1 = PARKING_SIGN_POSES[0][1];
			double distance_to_parking_spot = std::sqrt(std::pow((world.x_current[0] - x1), 2) + std::pow((world.x_current[1] - y1), 2));
			double detected_dist = utils.object_distance(park_index);
			double abs_error = std::abs(detected_dist - distance_to_parking_spot);
			if (abs_error < 1.) {
				// utils.debug("parking sign detected, proceeding to parking...", 3);
				return true;
			} else {
				utils.debug("WARNING: parking sign detected, but detected distance too large: " + std::to_string(detected_dist) +
								", expected: " + std::to_string(distance_to_parking_spot) + ", relocalizing...",
							2);
			}
		}
	}
	return false;
}

bool MoveForward::detect_obstacles() {
	double dist;
	std::list<int> cars = utils.recent_car_indices;
	// std::cout << "number of cars detected: " << cars.size() << std::endl;
	utils.debug("CHECK_CAR(): number of cars detected: " + std::to_string(cars.size()), 5);
	int car_index = utils.object_index(OBJECT::CAR);
	if (car_index >= 0) { // if car detected
		return true;
	}
	return false;
}

bool MoveForward::destination_reached() {
	double error_sq = (x_current.head(2) - world.destination).squaredNorm();
	if (error_sq < TOLERANCE_SQUARED && path_manager.target_waypoint_index >= path_manager.state_refs.rows() * 0.9) {
		return true;
	}
	return false;
}

void MoveForward::update_post_conditions() {
	post_conditions[STOP_SIGN_DETECTED] = detect_stop_sign();
	post_conditions[TRAFFIC_LIGHT_DETECTED] = detect_traffic_light();
    if (detect_parking_sign()) {
        post_conditions[PARKING_SIGN_DETECTED] = true;
    }
	post_conditions[OBSTACLE_DETECTED] = detect_obstacles();
	post_conditions[DESTINATION_REACHED] = destination_reached();
}
