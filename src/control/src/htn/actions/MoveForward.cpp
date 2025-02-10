#include "htn/actions/MoveForward.hpp"
#include "htn/Action.hpp"
#include <unordered_map>

MoveForward::MoveForward(World &world, std::unordered_map<PRIMITIVES, ValueType> &conditions) : Action(world, conditions) {
    cost = 1; // Moving forward is the best move as it moves us closer to the goal.
	pre_conditions = {
        {FORCE_STOP, false},
		{PARKING_SIGN_DETECTED, '_'},
        {PARKING_COUNT, '_'},
        {TRAFFIC_LIGHT_DETECTED, false},
        {STOP_SIGN_DETECTED, false},
        {OBSTACLE_DETECTED, false},
        {DESTINATION_REACHED, false},
	};
}

MoveForward::~MoveForward() {}

void MoveForward::execute() {
	if (!can_execute()) {
        utils.debug("Illegal action, pre conditions not satisfied", 2);
		return;
	}
    // Move the car forward
    utils.debug("Performing Action: Move Forward.", 2);
	world.update_mpc_states();
	solve();
	update_post_conditions();
	// TODO : Lane based relocalization
	// TODO : Sign based relocalization
}

void MoveForward::solve() {
	int success = path_manager.find_next_waypoint(path_manager.target_waypoint_index, x_current);
	// std::cout << "current state: x: " << x_current(0) << ", y: " << x_current(1) << ", yaw: " << x_current(2) << std::endl;
	// std::cout << "closest waypoint index: " << path_manager.closest_waypoint_index << ", at x: " << path_manager.state_refs(path_manager.closest_waypoint_index, 0) << ", y: " <<
	// path_manager.state_refs(path_manager.closest_waypoint_index, 1) << ", yaw: " << path_manager.state_refs(path_manager.closest_waypoint_index, 2) << std::endl; std::cout <<
	// "target waypoint index: " << path_manager.target_waypoint_index << ", at x: " << path_manager.state_refs(path_manager.target_waypoint_index, 0) << ", y: " <<
	// path_manager.state_refs(path_manager.target_waypoint_index, 1) << ", yaw: " << path_manager.state_refs(path_manager.target_waypoint_index, 2) << std::endl; for (int i =
	// path_manager.target_waypoint_index; i < std::min(path_manager.target_waypoint_index + 6, static_cast<int>(path_manager.state_refs.rows())); i++) {
	//     std::cout << "i: " << i << ", x: " << path_manager.state_refs(i, 0) << ", y: " << path_manager.state_refs(i, 1) << ", yaw: " << path_manager.state_refs(i, 2) <<
	//     std::endl;
	// }
	int idx = path_manager.target_waypoint_index;
	if (idx > path_manager.state_refs.rows() - 2) {
		idx = path_manager.state_refs.rows() - 2;
		utils.debug("WARNING: solve(): target waypoint index exceeds state_refs size, using last waypoint...", 3);
	}
	int N = std::min(Eigen::Index(path_manager.N), path_manager.state_refs.rows() - idx);
	if (idx >= 0 && idx <= path_manager.state_refs.rows() - 2 && N > 0) {

		Eigen::Block<Eigen::MatrixXd> state_refs_block = path_manager.state_refs.block(idx, 0, N, 3);
		Eigen::Block<Eigen::MatrixXd> input_refs_block = path_manager.input_refs.block(idx, 0, N, 2);
		int status = world.mpc.solve(state_refs_block, input_refs_block, x_current);
	} else {
		ROS_WARN("Block indices are out of bounds, skipping solve.");
		ROS_INFO("state_refs rows: %ld, cols: %ld", path_manager.state_refs.rows(), path_manager.state_refs.cols());
		ROS_INFO("input_refs rows: %ld, cols: %ld", path_manager.input_refs.rows(), path_manager.input_refs.cols());
		ROS_INFO("idx: %d, N: %d", idx, N);
	}
	world.publish_commands();
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
			double distance_to_parking_spot = std::sqrt(std::pow((x_current[0] - x1), 2) + std::pow((x_current[1] - y1), 2));
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
	current_state[STOP_SIGN_DETECTED] = detect_stop_sign();
	current_state[TRAFFIC_LIGHT_DETECTED] = detect_traffic_light();
    if (detect_parking_sign()) {
        current_state[PARKING_SIGN_DETECTED] = true;
    }
	current_state[OBSTACLE_DETECTED] = detect_obstacles();
	current_state[DESTINATION_REACHED] = destination_reached();
}
