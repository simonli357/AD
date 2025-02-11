#include "htn/World.hpp"
#include "ForceStop.hpp"
#include "MoveForward.hpp"
#include "ObstacleStop.hpp"
#include "Park.hpp"
#include "StopSignStop.hpp"
#include "TrafficLightStop.hpp"
#include "htn/Action.hpp"
#include "htn/HTN.hpp"

World::World(ros::NodeHandle &nh_, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name, double x_init, double y_init, double yaw_init, bool real)
	: nh(nh_), utils(nh, real, x_init, y_init, yaw_init, sign, ekf, lane, robot_name), mpc(T, N, v_ref), path_manager(nh, T, N, v_ref), sign(sign), ekf(ekf), lane(lane), T_park(T_park), T(T), real(real) {

	services_thread = std::thread(&World::receive_services, this);
    utils_thread = std::thread(&Utility::spin, &utils);

    initial_state = {
        {FORCE_STOP, true},
        {PARKING_SIGN_DETECTED, false},
        {PARKING_COUNT, 0},
        {TRAFFIC_LIGHT_DETECTED, false},
        {STOP_SIGN_DETECTED, false},
        {OBSTACLE_DETECTED, false},
        {DESTINATION_REACHED, false},
    };

    goal_state = {
        {DESTINATION_REACHED, true},
    };

	std::string mode = real ? "real" : "sim";
	utils.debug("mode: " + mode, 2);
	bool success = true;
	success = success && nh.getParam("/" + mode + "/change_lane_yaw", change_lane_yaw);
	success = success && nh.getParam("/" + mode + "/cw_speed_ratio", cw_speed_ratio);
	success = success && nh.getParam("/" + mode + "/hw_speed_ratio", hw_speed_ratio);
	success = success && nh.getParam("/" + mode + "/sign_localization_threshold", sign_localization_threshold);
	success = success && nh.getParam("/" + mode + "/lane_localization_orientation_threshold", lane_localization_orientation_threshold);
	success = success && nh.getParam("/" + mode + "/pixel_center_offset", pixel_center_offset);
	success = success && nh.getParam("/" + mode + "/constant_distance_to_intersection_at_detection", constant_distance_to_intersection_at_detection);
	success = success && nh.getParam("/" + mode + "/intersection_localization_threshold", intersection_localization_threshold);
	success = success && nh.getParam("/" + mode + "/stop_duration", stop_duration);
	success = success && nh.getParam("/" + mode + "/use_stopline", use_stopline);
	success = success && nh.getParam("/" + mode + "/pedestrian_count_thresh", pedestrian_count_thresh);
	success = success && nh.getParam("/" + mode + "/parking_base_yaw_target", parking_base_yaw_target);
	success = success && nh.getParam("/" + mode + "/parking_base_speed", parking_base_speed);
	success = success && nh.getParam("/" + mode + "/parking_base_thresh", parking_base_thresh);
	success = success && nh.getParam("/" + mode + "/change_lane_speed", change_lane_speed);
	success = success && nh.getParam("/" + mode + "/change_lane_thresh", change_lane_thresh);
	success = success && nh.getParam("/" + mode + "/intersection_localization_orientation_threshold", intersection_localization_orientation_threshold);
	success = success && nh.getParam("/" + mode + "/sign_localization_orientation_threshold", sign_localization_orientation_threshold);
	success = success && nh.getParam("/" + mode + "/NORMAL_SPEED", NORMAL_SPEED);
	success = success && nh.getParam("/" + mode + "/FAST_SPEED", FAST_SPEED);
	success = success && nh.getParam("/" + mode + "/lane_relocalize", lane_relocalize);
	success = success && nh.getParam("/" + mode + "/sign_relocalize", sign_relocalize);
	success = success && nh.getParam("/" + mode + "/intersection_relocalize", intersection_relocalize);
	success = success && nh.getParam("/" + mode + "/has_light", has_light);
	success = success && nh.getParam("/" + mode + "/change_lane_offset_scaler", change_lane_offset_scaler);
	success = success && nh.getParam("/emergency", emergency);
	success = success && nh.getParam("/pub_wpts", pubWaypoints);
	success = success && nh.getParam("/kb", keyboardControl);
	success = success && nh.getParam("/dashboard", dashboard);
	success = success && nh.getParam("/gps", hasGps);

	if (!success) {
		ROS_ERROR("Failed to get parameters");
		ros::shutdown();
	}

	// initialize parking spots
	for (int i = 0; i < 5; i++) {
		Eigen::Vector2d spot_right = {PARKING_SPOT_RIGHT[0] + i * PARKING_SPOT_LENGTH, PARKING_SPOT_RIGHT[1]};
		Eigen::Vector2d spot_left = {PARKING_SPOT_LEFT[0] + i * PARKING_SPOT_LENGTH, PARKING_SPOT_LEFT[1]};
		PARKING_SPOTS.push_back(spot_right);
		PARKING_SPOTS.push_back(spot_left);
	}

	double rateVal = 1 / mpc.T;
	rate = new ros::Rate(rateVal);
	std::cout << "rate: " << rateVal << std::endl;
	goto_command_server = nh.advertiseService("/goto_command", &World::goto_command_callback, this);
	set_states_server = nh.advertiseService("/set_states", &World::set_states_callback, this);
	start_trigger = nh.advertiseService("/start_bool", &World::start_bool_callback, this);
	utils.debug("start_bool server ready, mpc time step T = " + std::to_string(T), 2);
	utils.debug("world initialized", 2);
    initialize();
    htn_algorithm();
}

World::~World() {
	if (services_thread.joinable()) {
		services_thread.join();
	}
}

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
		utils.debug("intersection based relocalization(): SUCCESS: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " + std::to_string(estimated_position[1]) +
						"), actual: (" + std::to_string(direction_intersections[min_index][0]) + ", " + std::to_string(direction_intersections[min_index][1]) + "), error: (" +
						std::to_string(direction_intersections[min_index][0] - estimated_position[0]) + ", " + std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
					2);
		utils.recalibrate_states(direction_intersections[min_index][0] - estimated_position[0], direction_intersections[min_index][1] - estimated_position[1]);
		return 1; // Successful relocalization
	} else {
		utils.debug("intersection based relocalization(): FAILURE: estimated intersection position: (" + std::to_string(estimated_position[0]) + ", " + std::to_string(estimated_position[1]) +
						"), actual: (" + std::to_string(direction_intersections[min_index][0]) + ", " + std::to_string(direction_intersections[min_index][1]) + "), error: (" +
						std::to_string(direction_intersections[min_index][0] - estimated_position[0]) + ", " + std::to_string(direction_intersections[min_index][1] - estimated_position[1]) + ")",
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
					utils.debug("intersection_reached(): intersection detected, but distance (" + std::to_string(std::sqrt(dist_sq)) + ") too close to previous intersection, ignoring...", 4);
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

void World::call_trigger_service() {
	ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/trigger_service");
	std_srvs::Trigger srv;
	client.call(srv);
}

// ------------------//
// Private functions //
// ------------------//

int World::initialize() {
    if (initialized) return 1;
    // sleep 2 seconds to allow for initialization
    ros::Duration(2).sleep();
    if(hasGps) {
        if(!utils.reinitialize_states()) ROS_WARN("Failed to reinitialize");
    }
    double x, y, yaw;
    utils.get_states(x, y, yaw);
    utils.update_states(x_current);
    utils.debug("start(): x=" + std::to_string(x) + ", y=" + std::to_string(y) + ", yaw=" + std::to_string(yaw), 2);
    path_manager.call_waypoint_service(x, y, yaw, utils.tcp_client);
    destination = path_manager.state_refs.row(path_manager.state_refs.rows()-1).head(2);
    utils.debug("initialize(): start: " + std::to_string(x) + ", " + std::to_string(y), 2);
    utils.debug("initialize(): destination: " + std::to_string(destination(0)) + ", " + std::to_string(destination(1)), 2);

    path_manager.target_waypoint_index = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1); // search from the beginning to the end
    mpc.reset_solver();
    initialized = true;
    return 1;
}

void World::receive_services() {
	while (true) {
		if (utils.tcp_client == nullptr) {
			std::this_thread::sleep_for(std::chrono::milliseconds(10000));
			continue;
		}
		if (utils.tcp_client->get_go_to_cmd_srv_msgs().size() > 0) {
			double x = utils.tcp_client->get_go_to_cmd_srv_msgs().front()->dest_x;
			double y = utils.tcp_client->get_go_to_cmd_srv_msgs().front()->dest_y;
			utils::goto_command::Request req;
			utils::goto_command::Response res;
			req.dest_x = x;
			req.dest_y = y;
			goto_command_callback(req, res);
			utils.tcp_client->send_go_to_cmd_srv(res.state_refs, res.input_refs, res.wp_attributes, res.wp_normals, true);
			utils.tcp_client->get_go_to_cmd_srv_msgs().pop();
		}
		if (utils.tcp_client->get_set_states_srv_msgs().size() > 0) {
			double x = utils.tcp_client->get_set_states_srv_msgs().front()->x;
			double y = utils.tcp_client->get_set_states_srv_msgs().front()->y;
			utils::set_states::Request req;
			utils::set_states::Response res;
			req.x = x;
			req.y = y;
			set_states_callback(req, res);
			utils.tcp_client->send_set_states_srv(true);
			utils.tcp_client->get_set_states_srv_msgs().pop();
		}
		if (utils.tcp_client->get_start_srv_msgs().size() > 0) {
			std_srvs::SetBool::Request req;
			std_srvs::SetBool::Response res;
			req.data = utils.tcp_client->get_start_srv_msgs().front();
			start_bool_callback(req, res);
			utils.tcp_client->send_start_srv(true);
			utils.tcp_client->get_start_srv_msgs().pop();
		}
		if (utils.tcp_client->get_waypoints_srv_msgs().size() > 0) {
			double x0 = utils.tcp_client->get_waypoints_srv_msgs().front()->x0;
			double y0 = utils.tcp_client->get_waypoints_srv_msgs().front()->y0;
			double yaw0 = utils.tcp_client->get_waypoints_srv_msgs().front()->yaw0;
			path_manager.call_waypoint_service(x0, y0, yaw0, utils.tcp_client);
			utils.tcp_client->get_waypoints_srv_msgs().pop();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}
}

void World::htn_algorithm() {
    while (true) {
        current_state = initial_state;
        std::vector<std::unique_ptr<Action>> actions;
        actions.push_back(std::make_unique<ForceStop>(*this, current_state));
        actions.push_back(std::make_unique<MoveForward>(*this, current_state));
        actions.push_back(std::make_unique<ObstacleStop>(*this, current_state));
        actions.push_back(std::make_unique<Park>(*this, current_state));
        actions.push_back(std::make_unique<StopSignStop>(*this, current_state));
        actions.push_back(std::make_unique<TrafficLightStop>(*this, current_state));
        utils.debug("Starting htn algorithm", 2);
        HTN(*this, current_state, goal_state, actions).start();
    }
}

bool World::goto_command_callback(utils::goto_command::Request &req, utils::goto_command::Response &res) {
	utils.update_states(x_current);
	if (!path_manager.call_go_to_service(x_current[0], x_current[1], x_current[2], req.dest_x, req.dest_y)) {
		res.success = false;
		return false;
	}
	auto state_refs = path_manager.state_refs.transpose();
	auto input_refs = path_manager.input_refs.transpose();
	auto &state_attributes = path_manager.state_attributes;
	auto normals = path_manager.normals.transpose();
	res.state_refs.data = std::vector<float>(state_refs.data(), state_refs.data() + state_refs.size());
	res.input_refs.data = std::vector<float>(input_refs.data(), input_refs.data() + input_refs.size());
	res.wp_attributes.data = std::vector<float>(state_attributes.data(), state_attributes.data() + state_attributes.size());
	res.wp_normals.data = std::vector<float>(normals.data(), normals.data() + normals.size());
	res.success = true;
	destination = path_manager.state_refs.row(path_manager.state_refs.rows() - 1).head(2);
	// for (int i = 0; i<path_manager.state_refs.rows(); i++) {
	//     std::cout << i << ") " << path_manager.state_refs(i, 0) << ", " << path_manager.state_refs(i, 1) << ", " << path_manager.state_refs(i, 2) << std::endl;
	// }
	utils.debug("goto_command_callback(): start: " + std::to_string(x_current(0)) + ", " + std::to_string(x_current(1)), 2);
	utils.debug("goto_command_callback(): destination: " + std::to_string(destination(0)) + ", " + std::to_string(destination(1)), 2);

	path_manager.target_waypoint_index = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows() - 1); // search from the beginning to the end
	path_manager.overtake_end_index = 0;
	mpc.reset_solver();
	initialized = true;
	return true;
}

bool World::set_states_callback(utils::set_states::Request &req, utils::set_states::Response &res) {
	if (req.x >= 0 && req.y >= 0) {
		utils.set_states(req.x, req.y);
	} else {
		utils.reset_yaw();
	}
	res.success = true;
	return true;
}

bool World::start_bool_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
    auto& condition = current_state[FORCE_STOP];
    if (auto* value = std::get_if<bool>(&condition)) {
        if (*value) {
            current_state[FORCE_STOP] = false;
            res.success = true;
            res.message = "Started";
        } else {
            current_state[FORCE_STOP] = true;
            res.success = true;
            res.message = "Stopped";
        }
    }
	return true;
}
