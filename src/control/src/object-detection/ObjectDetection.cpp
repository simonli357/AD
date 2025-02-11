#include "object-detection/ObjectDetection.hpp"
#include "ValueTypes.hpp"
#include "World.hpp"
#include <unordered_map>

ObjectDetection::ObjectDetection(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state)
	: world(world), x_current(world.x_current), path_manager(world.path_manager), utils(world.utils), current_state(current_state) {
}

ObjectDetection::~ObjectDetection() {}

void ObjectDetection::detect_objects() {
    detect_signs();
    detect_pedestrians();
}

void ObjectDetection::detect_signs() {
	if (intersection_reached()) {
		switch (world.stopsign_flag) {
		case OBJECT::STOPSIGN:
			current_state[STOP_SIGN_DETECTED] = true;
			break;
		case OBJECT::LIGHTS:
			current_state[TRAFFIC_LIGHT_DETECTED] = true;
			break;
		default:
			break;
		}
        world.stopsign_flag = OBJECT::NONE;
	}
	check_road_signs();
	check_highway_signs();
	check_parking_signs();
}

void ObjectDetection::detect_pedestrians() {
	int pedestrian_count = 0;
	bool detected;
	if (world.real) {
		detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
	} else {
		// detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0 || utils.object_index(OBJECT::HIGHWAYEXIT) >= 0;
		detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
	}
	if (detected) {
		world.mpc.reset_solver();
        current_state[PEDESTRIAN_DETECTED] = true;
		while (true) {
			if (world.real) {
				detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
			} else {
				detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
			}
			if (detected) {
				double dist;
				if (world.real)
					dist = utils.object_distance(utils.object_index(OBJECT::HIGHWAYEXIT));
				else
					dist = utils.object_distance(utils.object_index(OBJECT::PEDESTRIAN));
				utils.debug("pedestrian_detected(): girl detected at a distance of: " + std::to_string(dist), 2);
			} else {
				pedestrian_count++;
				utils.debug("pedestrian_detected(): pedestrian sem: " + std::to_string(pedestrian_count) + " out of " + std::to_string(world.pedestrian_count_thresh), 2);
			}
			if (pedestrian_count > world.pedestrian_count_thresh)
				break;
		}
		world.rate->sleep();
        current_state[PEDESTRIAN_DETECTED] = false;
	}
}

void ObjectDetection::check_road_signs() {
	static bool relocalized = false;
	if (world.stopsign_flag != OBJECT::NONE && relocalized)
		return; // sign already detected
	if (world.stopsign_flag == OBJECT::NONE)
		relocalized = false;
	utils.update_states(x_current);
	double &x = x_current[0];
	double &y = x_current[1];
	double dist_sq = std::pow(x - world.last_intersection_point(0), 2) + std::pow(y - world.last_intersection_point(1), 2);
	if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD / 1.5 * INTERSECTION_DISTANCE_THRESHOLD / 1.5) {
		// distance to last intersection too close
		return;
	}

	int sign_index;
	double dist = -10.0;

	// check for stop sign
	sign_index = utils.object_index(OBJECT::STOPSIGN);
	if (world.stopsign_flag == OBJECT::NONE) { // if no sign detected
		if (sign_index >= 0) {
			dist = utils.object_distance(sign_index);
			if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
				world.detected_dist = dist;
				// if(lane) utils.reset_odom();
				if (sign_in_path(sign_index, dist + 0.2)) {
					utils.debug("check_stop_sign(): stop sign detected at a distance of: " + std::to_string(dist), 2);
					world.stopsign_flag = OBJECT::STOPSIGN;
				}
			}
		}
	}

	// check for traffic light
	if (world.stopsign_flag == OBJECT::NONE) {
		bool is_red = false;
		sign_index = utils.object_index(OBJECT::REDLIGHT);
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
			dist = utils.object_distance(sign_index);
			if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
				world.detected_dist = dist;
				// if(lane) utils.reset_odom();
				if (sign_in_path(sign_index, dist + 0.2)) {
					utils.debug("check_stop_sign(): traffic light detected at a distance of: " + std::to_string(dist), 2);
					world.stopsign_flag = OBJECT::LIGHTS;
				}
			}
		}
	}

	// check for priority sign
	if (world.stopsign_flag == OBJECT::NONE) {
		sign_index = utils.object_index(OBJECT::PRIORITY);
		if (sign_index >= 0) {
			dist = utils.object_distance(sign_index);
			if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
				world.detected_dist = dist;
				if (sign_in_path(sign_index, dist + 0.2)) {
					utils.debug("check_stop_sign(): priority detected at a distance of: " + std::to_string(dist), 2);
					world.stopsign_flag = OBJECT::PRIORITY;
				}
			}
		}
	}

	// check for roundabout sign
	if (world.stopsign_flag == OBJECT::NONE) {
		sign_index = utils.object_index(OBJECT::ROUNDABOUT);
		if (sign_index >= 0) {
			dist = utils.object_distance(sign_index);
			if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
				world.detected_dist = dist;
				if (sign_in_path(sign_index, dist + 0.2)) {
					utils.debug("check_stop_sign(): roundabout detected at a distance of: " + std::to_string(dist), 2);
					world.stopsign_flag = OBJECT::ROUNDABOUT;
				}
			}
		}
	}

	// check for crosswalk
	if (world.stopsign_flag == OBJECT::NONE) {
		sign_index = utils.object_index(OBJECT::CROSSWALK);
		if (sign_index >= 0) {
			dist = utils.object_distance(sign_index);
			if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
				world.detected_dist = dist;
				if (sign_in_path(sign_index, dist + 0.2)) {
					utils.debug("check_stop_sign(): crosswalk detected at a distance of: " + std::to_string(dist), 2);
					world.stopsign_flag = OBJECT::CROSSWALK;
				}
			}
		}
	}

	// relocalize based on sign
	if (world.sign_relocalize && world.stopsign_flag != OBJECT::NONE) {
		auto sign_pose = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], utils.object_box(sign_index), world.detected_dist);
		std::string sign_type;
		const auto &intersection_signs = utils.get_relevant_signs(world.stopsign_flag, sign_type);
		relocalized = world.sign_based_relocalization(sign_pose, intersection_signs, sign_type);
	}
}

bool ObjectDetection::check_highway_signs() {
	if (!world.sign_relocalize)
		return false;
	static ros::Time hw_cooldown_timer = ros::Time::now();
	static double detected_dist = -1;
	static int detection_count = 0;
	if (hw_cooldown_timer > ros::Time::now()) {
		return false;
	}
	bool is_exit = false;
	int hw_index = utils.object_index(OBJECT::HIGHWAYENTRANCE);
	if (hw_index < 0) {
		hw_index = utils.object_index(OBJECT::HIGHWAYEXIT);
		is_exit = true;
	}
	if (hw_index >= 0) {
		detected_dist = utils.object_distance(hw_index);
		// relocalize once at MAX_SIGN_DIST2, then again at MAX_SIGN_DIST
		double distance_threshold = detection_count == 0 ? MAX_SIGN_DIST2 : MAX_SIGN_DIST;
		if (detected_dist < distance_threshold && detected_dist > MIN_SIGN_DIST) {
			std::string hw_type = is_exit ? "exit" : "entrance";
			utils.update_states(x_current);
			auto hw_pose = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], utils.object_box(hw_index), detected_dist);
			std::string sign_type;
			const auto &direction_highways = utils.get_relevant_signs(is_exit ? OBJECT::HIGHWAYEXIT : OBJECT::HIGHWAYENTRANCE, sign_type);
			bool success = world.sign_based_relocalization(hw_pose, direction_highways, sign_type);
			// utils.
			detection_count++;
			if (detection_count >= 2) {
				double cd = (detected_dist + 4) / world.FAST_SPEED;
				hw_cooldown_timer = ros::Time::now() + ros::Duration(cd);
				detection_count = 0;
			}
			return success;
		}
	}
	return false;
}

void ObjectDetection::check_parking_signs() {
	int park_index = park_sign_detected();
	if (park_index >= 0) {
		auto x1 = PARKING_SIGN_POSES[0][0];
		auto y1 = PARKING_SIGN_POSES[0][1];
		double distance_to_parking_spot = std::sqrt(std::pow((x_current[0] - x1), 2) + std::pow((x_current[1] - y1), 2));
		double detected_dist = utils.object_distance(park_index);
		double abs_error = std::abs(detected_dist - distance_to_parking_spot);
		if (abs_error < 1.) {
			// utils.debug("parking sign detected, proceeding to parking...", 3);
			current_state[PARKING_SIGN_DETECTED] = true;
		} else {
			utils.debug("WARNING: parking sign detected, but detected distance too large: " + std::to_string(detected_dist) + ", expected: " + std::to_string(distance_to_parking_spot) + ", relocalizing...",
						2);
		}
	}
}

int ObjectDetection::park_sign_detected() {
	int park_index = utils.object_index(OBJECT::PARK);
	if (park_index >= 0) {
		double dist = utils.object_distance(park_index);
		if (dist < MAX_PARK_DIST && dist > 0) {
			world.detected_dist = dist;
			return park_index;
		}
	}
	return -1;
}

bool ObjectDetection::sign_in_path(int sign_idx, double search_dist) {
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

bool ObjectDetection::intersection_reached() {
	static double lookahead_dist = 0.15;
	static int num_index = static_cast<int>(lookahead_dist * path_manager.density);
	if (world.lane && world.use_stopline) {
		if (utils.stopline > 0) {
			utils.update_states(x_current);
			world.update_mpc_states(x_current[0], x_current[1], x_current[2]);
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
				double dist_sq = std::pow(x - world.last_intersection_point(0), 2) + std::pow(y - world.last_intersection_point(1), 2);
				if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
					utils.debug("intersection_reached(): intersection detected, but distance (" + std::to_string(std::sqrt(dist_sq)) + ") too close to previous intersection, ignoring...", 4);
					return false;
				}
				utils.debug("intersection_reached(): setting last intersection point to (" + std::to_string(x) + ", " + std::to_string(y) + ")", 2);
				world.last_intersection_point = {x, y};
				// find_next_intersection();
			} else {
				// utils.debug("intersection_reached(): found false, ignoring...", 2);
				return false;
			}
			// std::cout << "DEBUG: returning true" << std::endl;
			if (world.intersection_relocalize) {
				world.intersection_based_relocalization();
			}
			return true;
		} else
			return false;
	}
	utils.update_states(x_current);
	// utils.get_states(running_x, running_y, running_yaw);
	world.update_mpc_states(x_current[0], x_current[1], x_current[2]);
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
		double dist_sq = std::pow(x - world.last_intersection_point(0), 2) + std::pow(y - world.last_intersection_point(1), 2);
		if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
			// ROS_INFO("intersection detected, but too close to previous intersection: %.3f, ignoring...", std::sqrt(dist_sq));
			return false;
		}
		world.last_intersection_point = {x, y};
		utils.debug("intersection_reached(): not using lane, stopline waypoint reached.", 2);
		return true;
	}
	return false;
}

bool ObjectDetection::near_intersection() {
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
	estimated_position[0] += world.constant_distance_to_intersection_at_detection * cos(yaw);
	estimated_position[1] += world.constant_distance_to_intersection_at_detection * sin(yaw);

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
		utils.debug("near_intersection(): SUCCESS: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " + std::to_string(estimated_position[1]) + "), actual: (" +
						std::to_string(direction_intersections[min_index][0]) + ", " + std::to_string(direction_intersections[min_index][1]) + "), error: (" +
						std::to_string(direction_intersections[min_index][0] - estimated_position[0]) + ", " + std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
					4);
		return true;
	} else {
		utils.debug("near_intersection(): FAILURE: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " + std::to_string(estimated_position[1]) + "), actual: (" +
						std::to_string(direction_intersections[min_index][0]) + ", " + std::to_string(direction_intersections[min_index][1]) + "), error: (" +
						std::to_string(direction_intersections[min_index][0] - estimated_position[0]) + ", " + std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
					4);
		return false;
	}
}
