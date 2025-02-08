#include "MPC.hpp"
#include "PathManager.hpp"
#include "utility.hpp"
#include "utils/constants.h"

using namespace VehicleConstants;

class World {
  public:
	World(ros::NodeHandle &nh, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name, double x_init, double y_init, double yaw_init,
		  bool real);
	World(World &&) = delete;
	World(const World &) = delete;
	World &operator=(World &&) = delete;
	World &operator=(const World &) = delete;
	~World();

	ros::NodeHandle &nh;

	double change_lane_yaw = 0.15, cw_speed_ratio, hw_speed_ratio, sign_localization_threshold = 0.5, lane_localization_orientation_threshold = 10, pixel_center_offset = -30.0,
		   constant_distance_to_intersection_at_detection = 0.371, intersection_localization_threshold = 0.5, stop_duration = 3.0, parking_base_yaw_target = 0.166,
		   parking_base_speed = -0.2, parking_base_thresh = 0.1, change_lane_speed = 0.2, change_lane_thresh = 0.05, sign_localization_orientation_threshold = 15,
		   intersection_localization_orientation_threshold = 15, NORMAL_SPEED = 0.175, FAST_SPEED = 0.4, change_lane_offset_scaler = 1.2;
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

	void update_mpc_states(double x, double y, double yaw);
	void update_mpc_states();
	void solve();
	void publish_waypoints();
	void publish_commands();
	bool intersection_reached();
	int intersection_based_relocalization();
	bool near_intersection();
    bool sign_in_path(int sign_idx, double search_dist);

  private:
};
