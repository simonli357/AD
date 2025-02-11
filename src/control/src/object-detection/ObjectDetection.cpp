#include "object-detection/ObjectDetection.hpp"
#include "ValueTypes.hpp"
#include "World.hpp"
#include <unordered_map>

ObjectDetection::ObjectDetection(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state)
	: world(world), x_current(world.x_current), path_manager(world.path_manager), utils(world.utils), current_state(current_state) {
}

ObjectDetection::~ObjectDetection() {}

void ObjectDetection::detect_objects() {
    // detect objects and update the current state
    detect_signs();
    detect_pedestrians();
    detect_cars();
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

void ObjectDetection::detect_cars() {
    double dist;
    std::list<int> cars = utils.recent_car_indices;
    // std::cout << "number of cars detected: " << cars.size() << std::endl;
    utils.debug("CHECK_CAR(): number of cars detected: " + std::to_string(cars.size()), 5);
    int car_index = utils.object_index(OBJECT::CAR);
    if(car_index >= 0) { // if car detected
    // for (int car_index: cars) {
        utils.update_states(x_current);
        world.update_mpc_states(x_current[0], x_current[1], x_current[2]);
        utils.debug("CHECK_CAR(): current state: " + std::to_string(x_current[0]) + ", " + std::to_string(x_current[1]) + ", " + std::to_string(x_current[2]), 4);
        int closest_idx = path_manager.find_closest_waypoint(x_current);
        dist = utils.object_distance(car_index); // compute distance to back of car
        utils.debug("CHECK_CAR(): detected car at a distance of: " + std::to_string(dist) + ", closest index: " + std::to_string(closest_idx) + ", end index: " + std::to_string(path_manager.overtake_end_index), 4);
        double safety_dist = 0.3; // meters
        if (dist < MAX_CAR_DIST && dist > 0 && closest_idx >= path_manager.overtake_end_index + safety_dist * path_manager.density) { // if car is within range and ahead of ego car
            utils.object_box(car_index, world.bbox);
            double x, y, yaw;
            utils.get_states(x, y, yaw);
            auto car_pose = utils.estimate_object_pose2d(x, y, yaw, world.bbox, dist);
            // auto car_pose = utils.detected_cars[car_index];
            // compute distance from detected car to closest waypoint in front of car to assess whether car is in same lane
            double look_ahead_dist = dist * 1.5;
            int look_ahead_index = look_ahead_dist * path_manager.density + closest_idx;
            // compute distance from car_pose to waypoint, find closest waypoint and distance
            double min_dist_sq = 1000.;
            int min_index = 0;
            double min_dist_sq_adj = 1000.; // distance to adjacent lane
            int min_index_adj = 0;
            int idx = static_cast<int>(closest_idx + dist * path_manager.density * 0.75); // compute index of midpoint between detected car and ego car
            bool right = false;
            double start_dist = std::max(dist - CAM_TO_CAR_FRONT, MIN_DIST_TO_CAR) - MIN_DIST_TO_CAR;
            double density = path_manager.density;
            bool on_highway = false;
            static double lane_offset = LANE_OFFSET * world.change_lane_offset_scaler ;
            // if (attribute == path_manager.ATTRIBUTE::HIGHWAYRIGHT) { // if on right side of highway, overtake on left
            path_manager.overtake_end_index_scaler = 1.15;
            for (int i = idx; i < static_cast<int>(idx + 0.5 * path_manager.density); i++) {
                if (path_manager.attribute_cmp(i, path_manager.ATTRIBUTE::HIGHWAYRIGHT)) { // if on right side of highway, overtake on left
                    density *= 1/1.33;
                    path_manager.overtake_end_index_scaler *= 1.5;
                    on_highway = true;
                    utils.debug("CHECK_CAR(): detected car is on right side of highway, if overtake, on left", 2);
                    break;
                }
                // else if (attribute == path_manager.ATTRIBUTE::HIGHWAYLEFT) { // if on left side of highway, overtake on right
                else if (path_manager.attribute_cmp(i, path_manager.ATTRIBUTE::HIGHWAYLEFT)) { // if on left side of highway, overtake on right
                    right = true; 
                    on_highway = true;
                    density *= 1/1.33;
                    path_manager.overtake_end_index_scaler *= 1.5;
                    // utils.debug("CHECK_CAR(): detected car is on left side of highway, if overtake, on right", 3);
                    break;
                }
            }
            
            for (int i = closest_idx; i < look_ahead_index; i++) { // iterate over waypoints in front of car, compute distance to car
                // double dist_sq = (car_pose.head(2) - path_manager.state_refs.row(i).head(2)).squaredNorm();
                if (i >= path_manager.state_refs.rows()) {
                    utils.debug("CHECK_CAR(): WARNING: i exceeds state_refs size, stopping...", 2);
                    break;
                }
                double dist_sq = std::pow(car_pose[0] - path_manager.state_refs(i, 0), 2) + std::pow(car_pose[1] - path_manager.state_refs(i, 1), 2);
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    min_index = i;
                }
                int sign = right ? -1 : 1;
                // get adjacent lane point
                Eigen::Vector2d adj_point = (path_manager.state_refs.block(i, 0, 1, 2).transpose().eval() 
                        + (path_manager.normals.block(i, 0, 1, 2).transpose().eval() 
                        * LANE_OFFSET * sign));
                double dist_sq_adj = std::pow(car_pose[0] - adj_point(0), 2) + std::pow(car_pose[1] - adj_point(1), 2);
                if (dist_sq_adj < min_dist_sq_adj) {
                    min_dist_sq_adj = dist_sq_adj;
                    min_index_adj = i;
                }
            }
            double min_dist = std::sqrt(min_dist_sq);
            double min_dist_adj = std::sqrt(min_dist_sq_adj);
            auto detected_car_state = DETECTED_CAR_STATE::NOT_SURE;
            // if (min_dist < LANE_OFFSET - CAR_WIDTH * 0.8) {
            //     detected_car_state = DETECTED_CAR_STATE::SAME_LANE;
            // } else if (min_dist > LANE_OFFSET - CAR_WIDTH * 1.2) {
            //     detected_car_state = DETECTED_CAR_STATE::ADJACENT_LANE;
            // }
            utils.debug("CHECK_CAR(): min_dist: " + std::to_string(min_dist) + ", min_dist_adj: " + std::to_string(min_dist_adj), 4);
            if (on_highway && min_dist > LANE_OFFSET - CAR_WIDTH && min_dist_adj > LANE_OFFSET - CAR_WIDTH) {
                    detected_car_state = DETECTED_CAR_STATE::OPPOSITE_LANE;
            } else if (min_dist < LANE_OFFSET - CAR_WIDTH + SAME_LANE_SAFETY_FACTOR && min_dist < min_dist_adj) {
                detected_car_state = DETECTED_CAR_STATE::SAME_LANE;
            } else if (min_dist_adj < LANE_OFFSET - CAR_WIDTH && min_dist_adj < min_dist) {
                detected_car_state = DETECTED_CAR_STATE::ADJACENT_LANE;
            } else {
                detected_car_state = DETECTED_CAR_STATE::NOT_SURE;
            }
            // utils.debug("CHECK_CAR(): closest waypoint to detected car: " + std::to_string(min_index) + ", at " + std::to_string(path_manager.state_refs(min_index, 0)) + ", " + std::to_string(path_manager.state_refs(min_index, 1)), 3);
            // utils.debug("CHECK_CAR(): min dist between car and closest waypoint: " + std::to_string(min_dist) + ", same lane: " + std::to_string(detected_car_state == DETECTED_CAR_STATE::SAME_LANE), 3);
            if (detected_car_state == DETECTED_CAR_STATE::SAME_LANE) {
                if (idx < path_manager.state_refs.rows() && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::DOTTED) && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::DOTTED_CROSSWALK) && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::HIGHWAYLEFT) && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::HIGHWAYRIGHT)) {
                    if (dist < MAX_TAILING_DIST) {
                        current_state[CAR_DETECTED_ON_SAME_LANE] = true;
                        current_state[LANE_IS_DOTTED] = false;
                        return;
                    } else {
                        utils.debug("CHECK_CAR(): SAME_LANE: car on oneway pretty far and within safety margin, keep tailing: " + std::to_string(dist), 2);
                    }
                } else { // if detected car is in dotted region or on highway, we can overtake
                    int start_index = closest_idx + static_cast<int>(start_dist * density);
                    if (start_index >= path_manager.state_refs.rows() || path_manager.overtake_end_index >= path_manager.state_refs.rows()) {
                        utils.debug("CHECK_CAR(): WARNING: start or end index exceeds state_refs size, stopping...", 2);
                        return;
                    };
                    current_state[CAR_DETECTED_ON_SAME_LANE] = true;
                    current_state[LANE_IS_DOTTED] = true;
                    return;
                }
            } else if (detected_car_state == DETECTED_CAR_STATE::NOT_SURE) {
                if (dist < MAX_TAILING_DIST) {
                    utils.debug("CHECK_CAR(): NOT_SURE: dist = " + std::to_string(dist) + ", stopping... min_dist: " + std::to_string(min_dist) + ", min_dist_adj: " + std::to_string(min_dist_adj) + ", car pose: (" + std::to_string(car_pose[0]) + ", " + std::to_string(car_pose[1]) + ")", 2);
                    current_state[CAR_DETECTED_ON_SAME_LANE] = true;
                    current_state[LANE_IS_DOTTED] = false;
                    return;
                } else {
                    utils.debug("CHECK_CAR(): NOT_SURE: car pretty far and within safety margin, keep tailing: " + std::to_string(dist), 2);
                }
            } else if (detected_car_state == DETECTED_CAR_STATE::OPPOSITE_LANE) {
                return;
            }
        }
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
