#pragma once

#include "MPC.hpp"
#include "PathManager.hpp"
#include "htn/Primitives.hpp"
#include "htn/ValueTypes.hpp"
#include "utility.hpp"
#include "utils/constants.h"
#include "utils/goto_command.h"
#include "utils/set_states.h"
#include <std_srvs/SetBool.h>
#include <unordered_map>

using namespace VehicleConstants;

class World {
  public:
	World(ros::NodeHandle &nh, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name, double x_init, double y_init, double yaw_init, bool real);
	World(World &&) = delete;
	World(const World &) = delete;
	World &operator=(World &&) = delete;
	World &operator=(const World &) = delete;
	~World();

	std::unordered_map<PRIMITIVES, ValueType> initial_state;
	std::unordered_map<PRIMITIVES, ValueType> goal_state;
	std::unordered_map<PRIMITIVES, ValueType> current_state;

	ros::NodeHandle &nh;
	ros::ServiceServer start_trigger;

	double change_lane_yaw = 0.15, cw_speed_ratio, hw_speed_ratio, sign_localization_threshold = 0.5, lane_localization_orientation_threshold = 10, pixel_center_offset = -30.0,
		   constant_distance_to_intersection_at_detection = 0.371, intersection_localization_threshold = 0.5, stop_duration = 3.0, parking_base_yaw_target = 0.166, parking_base_speed = -0.2,
		   parking_base_thresh = 0.1, change_lane_speed = 0.2, change_lane_thresh = 0.05, sign_localization_orientation_threshold = 15, intersection_localization_orientation_threshold = 15,
		   NORMAL_SPEED = 0.175, FAST_SPEED = 0.4, change_lane_offset_scaler = 1.2;
	bool use_stopline = true, lane_relocalize = true, sign_relocalize = true, intersection_relocalize = true, has_light = false, emergency = false;
	bool initialized = false;
	int pedestrian_count_thresh = 8;

	Eigen::Vector3d x_current;
	std::vector<Eigen::Vector2d> PARKING_SPOTS;

	std::array<double, 4> bbox = {0.0, 0.0, 0.0, 0.0};
	double T_park, T;
	double detected_dist = 0;
	bool right_park = true;
	int park_count = 0;
	int stopsign_flag = OBJECT::NONE;
	Eigen::Vector2d destination;
	int state = 0;
	bool sign, ekf, lane, real, dashboard, keyboardControl, hasGps, pubWaypoints;

	ros::Rate *rate;

	std::mutex lock;
	Utility utils;
	PathManager path_manager;
	ros::ServiceServer goto_command_server, set_states_server;
	MPC mpc;

	// intersection variables
	Eigen::Vector2d last_intersection_point = {1000.0, 1000.0};
	Eigen::Vector2d next_intersection_point = {1000.0, 1000.0};

	void htn_algorithm();
	void update_mpc_states(double x, double y, double yaw);
	void update_mpc_states();
	void publish_waypoints();
	void publish_commands();
	void call_trigger_service();
    double check_crosswalk();
    double check_highway();
    int intersection_based_relocalization();
    int sign_based_relocalization(const Eigen::Vector2d estimated_sign_pose, const std::vector<std::vector<double>> &EMPIRICAL_POSES, const std::string &sign_type = "");

  private:
	std::thread services_thread;
	std::thread utils_thread;
	int initialize();
	void receive_services();
	bool goto_command_callback(utils::goto_command::Request &req, utils::goto_command::Response &res);
	bool goto_command_callback_tcp(std::vector<std::tuple<float, float>> &coords, utils::goto_command::Response &res);
	bool set_states_callback(utils::set_states::Request &req, utils::set_states::Response &res);
	bool start_bool_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res);
};
