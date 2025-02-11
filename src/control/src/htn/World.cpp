#include "htn/World.hpp"
#include "ForceStop.hpp"
#include "HTN.hpp"
#include "MoveForward.hpp"
#include "PedestrianStop.hpp"
#include "htn/actions/park/Park.hpp"
#include "StopSignStop.hpp"
#include "TrafficLightStop.hpp"
#include "htn/Action.hpp"

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
        {PEDESTRIAN_DETECTED, false},
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
    if (utils_thread.joinable()) {
        utils_thread.join();
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

void World::call_trigger_service() {
	ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/trigger_service");
	std_srvs::Trigger srv;
	client.call(srv);
}

double World::check_crosswalk() {
    static ros::Time crosswalk_cooldown_timer = ros::Time::now();
    static double detected_dist = -1;
    if(crosswalk_cooldown_timer > ros::Time::now()) {
        // std::cout << "crosswalk detected previously, cd expire in " << (crosswalk_cooldown_timer - ros::Time::now()).toSec() << "s" << std::endl;
        return 100;
    }
    int crosswalk_index = utils.object_index(OBJECT::CROSSWALK);
    if(crosswalk_index >= 0) {
        detected_dist = utils.object_distance(crosswalk_index);
        if (detected_dist < MAX_CROSSWALK_DIST && detected_dist > 0) {
            double cd = (detected_dist + CROSSWALK_LENGTH) / NORMAL_SPEED * cw_speed_ratio;
            crosswalk_cooldown_timer = ros::Time::now() + ros::Duration(cd);
            utils.debug("crosswalk detected at a distance of: " + std::to_string(detected_dist), 2);
            if (sign_relocalize) {
            // if (1) {
                utils.update_states(x_current);
                auto crosswalk_pose = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], utils.object_box(crosswalk_index), detected_dist);
                std::string sign_type;
                const auto& direction_crosswalks = utils.get_relevant_signs(OBJECT::CROSSWALK, sign_type);
                int nearestDirectionIndex = Utility::nearest_direction_index(x_current[2]);
                sign_based_relocalization(crosswalk_pose, direction_crosswalks, sign_type);
            }
            return detected_dist;
        }
    }
    return -1;
}

double World::check_highway() {
    utils.update_states(x_current);
    update_mpc_states(x_current[0], x_current[1], x_current[2]);
    int closest_idx = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1);
    return path_manager.attribute_cmp(closest_idx, path_manager.ATTRIBUTE::HIGHWAYLEFT) || path_manager.attribute_cmp(closest_idx, path_manager.ATTRIBUTE::HIGHWAYRIGHT);
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

int World::sign_based_relocalization(const Eigen::Vector2d estimated_sign_pose, const std::vector<std::vector<double>> &EMPIRICAL_POSES, const std::string &sign_type) {
	int min_index = 0;
	double min_error_sq = 1000.0;
	if (utils.get_min_object_index(estimated_sign_pose, EMPIRICAL_POSES, min_index, min_error_sq, sign_localization_threshold)) {
		double yaw = utils.get_yaw();
		double sign_direction = EMPIRICAL_POSES[min_index][2];
		double yaw_error = Utility::compare_yaw(sign_direction, yaw);
		if (yaw_error > sign_localization_orientation_threshold * M_PI / 180) {
			utils.debug("SIGN_RELOC(" + sign_type + "): FAILURE: yaw error too large: " + std::to_string(yaw_error) + ", threshold: " + std::to_string(sign_localization_orientation_threshold), 2);
			return 0;
		}
		utils.debug("SIGN_RELOC(" + sign_type + "): SUCCESS: estimated sign pose: (" + std::to_string(estimated_sign_pose[0]) + ", " + std::to_string(estimated_sign_pose[1]) + "), actual: (" +
						std::to_string(EMPIRICAL_POSES[min_index][0]) + ", " + std::to_string(EMPIRICAL_POSES[min_index][1]) + "), error: (" +
						std::to_string(EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0]) + ", " + std::to_string(EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]) +
						"), error norm: " + std::to_string(std::sqrt(min_error_sq)) + ", threshold: " + std::to_string(sign_localization_threshold),
					2);
		double x, y;
		utils.get_states(x, y, yaw);
		utils.debug("SIGN_RELOC(" + sign_type + "): relative estimated pose to car: (" + std::to_string(estimated_sign_pose[0] - x) + ", " + std::to_string(estimated_sign_pose[1] - y) + ")", 3);
		utils.recalibrate_states(EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0], EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]);
	} else {
		utils.debug("SIGN_RELOC(" + sign_type + "): FAILURE: error too large: " + std::to_string(std::sqrt(min_error_sq)) + ", threshold: " + std::to_string(sign_localization_threshold) +
						", estimated sign pose: (" + std::to_string(estimated_sign_pose[0]) + ", " + std::to_string(estimated_sign_pose[1]) + ")",
					2);
		return 0;
	}
	utils.update_states(x_current);
	path_manager.reset_target_waypoint_index(x_current);
	mpc.reset_solver();
	return 1;
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
        actions.push_back(std::make_unique<PedestrianStop>(*this, current_state));
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
