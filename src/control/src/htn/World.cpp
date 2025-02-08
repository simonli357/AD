#include "htn/World.hpp"

World::World(ros::NodeHandle &nh_, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name, double x_init, double y_init,
			 double yaw_init, bool real)
	: nh(nh_), utils(nh, real, x_init, y_init, yaw_init, sign, ekf, lane, robot_name), mpc(T, N, v_ref), path_manager(nh, T, N, v_ref), sign(sign), ekf(ekf), lane(lane),
	  T_park(T_park), T(T), real(real) {}

void World::update_mpc_states() {
	utils.update_states(x_current);
	double yaw = x_current(2);
	if (path_manager.closest_waypoint_index < path_manager.state_refs.rows() && path_manager.state_refs.rows() > 0) {
		double ref_yaw = path_manager.state_refs(path_manager.closest_waypoint_index, 2);
		yaw = Utility::yaw_mod(yaw, ref_yaw);
	}
	x_current(2) = yaw;
}

void World::update_mpc_states(double x, double y, double yaw) {
	if (path_manager.closest_waypoint_index < path_manager.state_refs.rows() && path_manager.state_refs.rows() > 0) {
		double ref_yaw = path_manager.state_refs(path_manager.closest_waypoint_index, 2);
		yaw = Utility::yaw_mod(yaw, ref_yaw);
	}
	x_current << x, y, yaw;
}

void World::solve() {
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
		int status = mpc.solve(state_refs_block, input_refs_block, x_current);
	} else {
		ROS_WARN("Block indices are out of bounds, skipping solve.");
		ROS_INFO("state_refs rows: %ld, cols: %ld", path_manager.state_refs.rows(), path_manager.state_refs.cols());
		ROS_INFO("input_refs rows: %ld, cols: %ld", path_manager.input_refs.rows(), path_manager.input_refs.cols());
		ROS_INFO("idx: %d, N: %d", idx, N);
	}
	publish_commands();
}

void World::publish_waypoints() {
	static Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(mpc.N, 3);
	path_manager.get_current_waypoints(waypoints);
	static std_msgs::Float32MultiArray msg;
	msg.data.clear();
	for (int i = 0; i < waypoints.rows(); ++i) {
		msg.data.push_back(waypoints(i, 0)); // x
		msg.data.push_back(waypoints(i, 1)); // y
	}
	utils.waypoints_pub.publish(msg);
	if (utils.tcp_client != nullptr)
		utils.tcp_client->send_waypoint(msg);
}

void World::publish_commands() {
	static int count = 0;
	if (pubWaypoints) {
		publish_waypoints();
	}
	double steer = -mpc.u_current[1] * 180 / M_PI;
	double speed = mpc.u_current[0];
	// std::cout << "steer: " << steer << ", speed: " << speed << std::endl;
	// std::cout << speed*100 << ", " << steer << std::endl;
	utils.publish_cmd_vel(steer, speed);
}

bool World::near_intersection() {
	double yaw = utils.get_yaw();
	double nearest_direction = Utility::nearest_direction(yaw);
	double yaw_error = nearest_direction - yaw;
	if (yaw_error > M_PI * 1.5)
		yaw_error -= 2 * M_PI;
	else if (yaw_error < -M_PI * 1.5)
		yaw_error += 2 * M_PI;
	if (std::abs(yaw_error) > 45 * M_PI / 180) {
		utils.debug("near_intersection(): FAILURE: yaw error too large: " + std::to_string(yaw_error), 4);
		return false;
	}

	int nearestDirectionIndex = Utility::nearest_direction_index(yaw);
	const auto &direction_intersections = (nearestDirectionIndex == 0)	 ? EAST_FACING_INTERSECTIONS
										  : (nearestDirectionIndex == 1) ? NORTH_FACING_INTERSECTIONS
										  : (nearestDirectionIndex == 2) ? WEST_FACING_INTERSECTIONS
																		 : SOUTH_FACING_INTERSECTIONS;

	static Eigen::Vector2d estimated_position(0, 0);
	utils.get_states(estimated_position(0), estimated_position(1), yaw);
	estimated_position[0] += constant_distance_to_intersection_at_detection * cos(yaw);
	estimated_position[1] += constant_distance_to_intersection_at_detection * sin(yaw);

	double min_error_sq = std::numeric_limits<double>::max();
	int min_index = 0;
	for (size_t i = 0; i < direction_intersections.size(); ++i) {
		double error_sq = std::pow(estimated_position[0] - direction_intersections[i][0], 2) + std::pow(estimated_position[1] - direction_intersections[i][1], 2);
		if (error_sq < min_error_sq) {
			min_error_sq = error_sq;
			min_index = static_cast<int>(i);
		}
	}
	// exit(0);
	if (min_error_sq < 0.3 * 0.3) {
		utils.debug("near_intersection(): SUCCESS: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " + std::to_string(estimated_position[1]) +
						"), actual: (" + std::to_string(direction_intersections[min_index][0]) + ", " + std::to_string(direction_intersections[min_index][1]) + "), error: (" +
						std::to_string(direction_intersections[min_index][0] - estimated_position[0]) + ", " +
						std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
					4);
		return true;
	} else {
		utils.debug("near_intersection(): FAILURE: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " + std::to_string(estimated_position[1]) +
						"), actual: (" + std::to_string(direction_intersections[min_index][0]) + ", " + std::to_string(direction_intersections[min_index][1]) + "), error: (" +
						std::to_string(direction_intersections[min_index][0] - estimated_position[0]) + ", " +
						std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
					4);
		return false;
	}
}

int World::intersection_based_relocalization() {
	// stop_for(0.5);
	// check orientation
	double yaw = utils.get_yaw();
	double nearest_direction = Utility::nearest_direction(yaw);
	double yaw_error = nearest_direction - yaw;
	if (yaw_error > M_PI * 1.5)
		yaw_error -= 2 * M_PI;
	else if (yaw_error < -M_PI * 1.5)
		yaw_error += 2 * M_PI;
	if (std::abs(yaw_error) > intersection_localization_orientation_threshold * M_PI / 180) {
		utils.debug("intersection_based_relocalization(): FAILURE: yaw error too large: " + std::to_string(yaw_error), 2);
		return 0;
	}

	int nearestDirectionIndex = Utility::nearest_direction_index(yaw);
	const auto &direction_intersections = (nearestDirectionIndex == 0)	 ? EAST_FACING_INTERSECTIONS
										  : (nearestDirectionIndex == 1) ? NORTH_FACING_INTERSECTIONS
										  : (nearestDirectionIndex == 2) ? WEST_FACING_INTERSECTIONS
																		 : SOUTH_FACING_INTERSECTIONS;

	static Eigen::Vector2d estimated_position(0, 0);
	utils.get_states(estimated_position(0), estimated_position(1), yaw);
	estimated_position[0] += constant_distance_to_intersection_at_detection * cos(yaw);
	estimated_position[1] += constant_distance_to_intersection_at_detection * sin(yaw);

	double min_error_sq = std::numeric_limits<double>::max();
	int min_index = 0;
	for (size_t i = 0; i < direction_intersections.size(); ++i) {
		double error_sq = std::pow(estimated_position[0] - direction_intersections[i][0], 2) + std::pow(estimated_position[1] - direction_intersections[i][1], 2);
		if (error_sq < min_error_sq) {
			min_error_sq = error_sq;
			min_index = static_cast<int>(i);
		}
	}

	// exit(0);
	if (min_error_sq < intersection_localization_threshold * intersection_localization_threshold) {
		utils.debug("intersection based relocalization(): SUCCESS: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " +
						std::to_string(estimated_position[1]) + "), actual: (" + std::to_string(direction_intersections[min_index][0]) + ", " +
						std::to_string(direction_intersections[min_index][1]) + "), error: (" + std::to_string(direction_intersections[min_index][0] - estimated_position[0]) +
						", " + std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
					2);
		utils.recalibrate_states(direction_intersections[min_index][0] - estimated_position[0], direction_intersections[min_index][1] - estimated_position[1]);
		return 1; // Successful relocalization
	} else {
		utils.debug("intersection based relocalization(): FAILURE: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " +
						std::to_string(estimated_position[1]) + "), actual: (" + std::to_string(direction_intersections[min_index][0]) + ", " +
						std::to_string(direction_intersections[min_index][1]) + "), error: (" + std::to_string(direction_intersections[min_index][0] - estimated_position[0]) +
						", " + std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
					2);
		return 0; // Failed to relocalize
	}
}

bool World::intersection_reached() {
	static double lookahead_dist = 0.15;
	static int num_index = static_cast<int>(lookahead_dist * path_manager.density);
	if (lane && use_stopline) {
		if (utils.stopline > 0) {
			utils.update_states(x_current);
			update_mpc_states(x_current[0], x_current[1], x_current[2]);
			// int closest_idx = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1);
			// int num_index = static_cast<int>(0.15 * path_manager.density);
			// for (int i = closest_idx; i < closest_idx + num_index; i++) {
			//     if (i >= path_manager.state_refs.rows()) break;
			//     if (path_manager.attribute_cmp(i, path_manager.ATTRIBUTE::CROSSWALK) || path_manager.attribute_cmp(i, path_manager.ATTRIBUTE::DOTTED_CROSSWALK)) {
			//         utils.debug("intersection_reached(): waypoint attribute is crosswalk, ignoring...", 2);
			//         return false;
			//     }
			// }

			// if(check_crosswalk() > 0) {
			//     utils.debug("intersection_reached(): detected crosswalk, ignoring...", 2);
			//     return false;
			// }

			bool found = false;
			// lookahead_dist = 0.8;
			// double lookbehind_dist = 0.3;
			// num_index = static_cast<int>(lookahead_dist * path_manager.density);
			// int lookbehind_index = static_cast<int>(lookbehind_dist * path_manager.density);
			// for (int i = -lookbehind_index; i < num_index; i++) {
			//     if (closest_idx + i >= path_manager.state_refs.rows()) {
			//         utils.debug("intersection_reached(): closest idx + i = " + std::to_string(closest_idx + i) + " exceeds path_manager.state_refs.rows(): " +
			//         std::to_string(path_manager.state_refs.rows()), 2); break;
			//     }
			//     if (closest_idx + i < 0) continue;
			//     // ROS_INFO("checking index %d at (%.2f, %.2f)", closest_idx + i, path_manager.state_refs(closest_idx + i, 0), path_manager.state_refs(closest_idx + i, 1));
			//     if(path_manager.attribute_cmp(closest_idx+i, path_manager.ATTRIBUTE::STOPLINE)) {
			//         found = true;
			//         break;
			//     }
			// }
			if (near_intersection()) {
				found = true;
			}
			if (found) {
				double &x = x_current[0];
				double &y = x_current[1];
				double dist_sq = std::pow(x - last_intersection_point(0), 2) + std::pow(y - last_intersection_point(1), 2);
				if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
					utils.debug("intersection_reached(): intersection detected, but distance (" + std::to_string(std::sqrt(dist_sq)) +
									") too close to previous intersection, ignoring...",
								4);
					return false;
				}
				utils.debug("intersection_reached(): setting last intersection point to (" + std::to_string(x) + ", " + std::to_string(y) + ")", 2);
				last_intersection_point = {x, y};
				// find_next_intersection();
			} else {
				// utils.debug("intersection_reached(): found false, ignoring...", 2);
				return false;
			}
			// std::cout << "DEBUG: returning true" << std::endl;
			if (intersection_relocalize) {
				intersection_based_relocalization();
			}
			return true;
		} else
			return false;
	}
	utils.update_states(x_current);
	// utils.get_states(running_x, running_y, running_yaw);
	update_mpc_states(x_current[0], x_current[1], x_current[2]);
	int target_index = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows() - 1);
	bool found = false;
	for (int i = 0; i < num_index; i++) {
		if (target_index + i >= path_manager.state_refs.rows())
			break;
		if (path_manager.attribute_cmp(target_index + i, path_manager.ATTRIBUTE::STOPLINE)) {
			found = true;
			break;
		}
	}
	if (found) {
		double x, y, yaw;
		utils.get_states(x, y, yaw);
		double dist_sq = std::pow(x - last_intersection_point(0), 2) + std::pow(y - last_intersection_point(1), 2);
		if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
			// ROS_INFO("intersection detected, but too close to previous intersection: %.3f, ignoring...", std::sqrt(dist_sq));
			return false;
		}
		last_intersection_point = {x, y};
		utils.debug("intersection_reached(): not using lane, stopline waypoint reached.", 2);
		return true;
	}
	return false;
}

bool World::sign_in_path(int sign_idx, double search_dist) {
	auto estimated_sign_pose = utils.object_world_pose(sign_idx);
	double x = estimated_sign_pose[0];
	double y = estimated_sign_pose[1];
	int closest_idx = path_manager.closest_waypoint_index;
	int num_index = static_cast<int>(search_dist * path_manager.density);
	double min_dist_sq = std::numeric_limits<double>::max();
	double threshold = INTERSECTION_TO_SIGN * INTERSECTION_TO_SIGN * 1.5 * 1.5;
	for (int i = closest_idx; i < closest_idx + num_index; i += 4) {
		if (i >= path_manager.state_refs.rows())
			break;
		double dist_sq = std::pow(x - path_manager.state_refs(i, 0), 2) + std::pow(y - path_manager.state_refs(i, 1), 2);
		if (dist_sq < min_dist_sq) {
			min_dist_sq = dist_sq;
		}
		if (min_dist_sq < threshold) {
			utils.debug("sign_in_path(): sign at (" + std::to_string(x) + ", " + std::to_string(y) + ") found in path", 1);
			return true;
		}
	}
	return false;
}
