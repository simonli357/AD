#pragma once

#include "GroundTruth.h"
#include "PathPlanner.hpp"
#include "TcpClient.hpp"
#include "Utility.hpp"
#include "utils/helper.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/TriggerResponse.h"
#include "utils/Point2D.h"
#include "utils/constants.h"
#include "utils/go_to.h"
#include "utils/go_to_multiple.h"
#include "utils/waypoints.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <thread>
#include <vector>
#include <string>
#include <atomic>

namespace PathManager {

inline std::atomic<bool> path_manager_initialized = false;
inline ros::NodeHandle* nh = nullptr;
inline PathPlanner path_planner{0.25, 40, 0.125};

inline double T = 0.125;
inline int N = 40;
inline double v_ref = 0.25;
inline std::string pathName = "speedrun";

inline double density = 1.0 / T / v_ref;
inline double region_of_acceptance = 0.03076923 * 3 * (0.125 * 1.3) / density;
inline double region_of_acceptance_cw = region_of_acceptance / 1.5;
inline double region_of_acceptance_hw = region_of_acceptance * 1.5;

inline double t0 = 0.0;
inline int closest_waypoint_index = 0;
inline int target_waypoint_index = 0;
inline int last_waypoint_index = 0;
inline int overtake_end_index = 0;
inline int overtake_end_index_scaler = 1.15;
inline double rdb_circumference = 3.95;
inline bool debug = true;

inline int v_ref_int = static_cast<int>(v_ref * 100);

inline ros::ServiceClient waypoints_client;
inline ros::ServiceClient go_to_client;
inline ros::ServiceClient go_to_multiple_client;
inline ros::ServiceClient trigger_client;

inline Eigen::MatrixXd state_refs;
inline Eigen::MatrixXd input_refs;
inline Eigen::MatrixXd normals;
inline Eigen::MatrixXd left_turn_states;
inline Eigen::MatrixXd right_turn_states;
inline Eigen::MatrixXd straight_states;
inline Eigen::VectorXd state_attributes;
inline Eigen::MatrixXd* state_refs_ptr = &state_refs;

inline std::vector<int> intersection_indices;
inline int intersection_index = 0;

enum ATTRIBUTE { NORMAL, CROSSWALK, INTERSECTION, ONEWAY, HIGHWAYLEFT, HIGHWAYRIGHT, ROUNDABOUT, STOPLINE, DOTTED, DOTTED_CROSSWALK };

inline void init(ros::NodeHandle& nh_, double T_, int N_, double v_ref_, const std::string& pathName_) {
    nh = &nh_;
    T = T_; N = N_; v_ref = v_ref_; pathName = pathName_;
    density = 1.0 / T / v_ref;
    region_of_acceptance = 0.03076923 * 3 * (0.125 * 1.3) / density;
    region_of_acceptance_cw = region_of_acceptance / 1.5;
    region_of_acceptance_hw = region_of_acceptance * 1.5;
    t0 = 0.0;
    closest_waypoint_index = target_waypoint_index = last_waypoint_index = 0;
    v_ref_int = static_cast<int>(v_ref * 100);
    path_planner = PathPlanner(v_ref, N, T);
    waypoints_client       = nh->serviceClient<utils::waypoints>  ("/waypoint_path");
    go_to_client           = nh->serviceClient<utils::go_to>       ("/go_to");
    go_to_multiple_client  = nh->serviceClient<utils::go_to_multiple>("/go_to_multiple");
    trigger_client         = nh->serviceClient<std_srvs::Trigger>  ("/notify_params_updated");
		path_manager_initialized = true;
}

inline void init(ros::NodeHandle& nh_) {
    init(nh_, 0.125, 40, 0.25, "speedrun");
}

inline bool find_intersections(Utility& utils) {
	using namespace GroundTruth;
	utils.debug("FIND_INTERSECTIONS(): started", 1);

	auto start = std::chrono::high_resolution_clock::now();
	intersection_indices.clear();
	intersection_index = 0;

	if (state_refs.rows() == 0) {
		utils.debug("FIND_INTERSECTIONS(): FAILURE: state_refs is empty", 1);
		return false;
	}

	int current_idx = 0;
	double threshold = INTERSECTION_DISTANCE_THRESHOLD;

	std::cout << "FIND_INTERSECTIONS(): state_refs size: " << state_refs.rows() << ", threshold: " << threshold << ", density: " << density << ", T: " << T << ", v_ref: " << v_ref << std::endl;
	while (current_idx < state_refs.rows()) {
		double x = state_refs(current_idx, 0);
		double y = state_refs(current_idx, 1);
		double yaw = state_refs(current_idx, 2);

		int nearest_dir_idx = helper::nearest_direction_index(yaw);
		double dir_yaw = helper::nearest_direction(yaw);
		double yaw_error = helper::compare_yaw(yaw, dir_yaw);

		if (yaw_error * 180 / M_PI > 15) {
			current_idx += static_cast<int>(density * threshold * 0.33 / 2);
			continue;
		}

		// Check all intersections for the nearest
		double min_error_sq = std::numeric_limits<double>::max();
		int closest_idx = -1;
		for (size_t i = 0; i < intersections_all.size(); ++i) {
			double yaw_error2 = helper::compare_yaw(dir_yaw, intersections_all[i].pose[2]);
			if (yaw_error2 * 180 / M_PI > 1) {
				continue;
			}
			double dx = x - intersections_all[i].pose[0];
			double dy = y - intersections_all[i].pose[1];
			double dist_sq = dx * dx + dy * dy;

			if (dist_sq < min_error_sq) {
				min_error_sq = dist_sq;
				closest_idx = static_cast<int>(i);
			}
		}

		if (closest_idx >= 0 && min_error_sq < threshold * threshold) {
			// Avoid adding same intersection back-to-back
			bool same_as_last = false;
			if (!intersection_indices.empty()) {
				int last_idx = intersection_indices.back();
				const auto &last_pose = intersections_all[last_idx].pose.head<2>();
				const auto &this_pose = intersections_all[closest_idx].pose.head<2>();
				double dist_to_last_sq = (last_pose - this_pose).squaredNorm();
				same_as_last = dist_to_last_sq < threshold * threshold * 0.75 * 0.75;
			}

			if (!same_as_last) {
				Eigen::Vector2d current(x, y);
				Eigen::Vector2d intersection_pos(intersections_all[closest_idx].pose[0], intersections_all[closest_idx].pose[1]);
				Eigen::Vector2d vec_to_intersection = intersection_pos - current;

				Eigen::Vector2d path_dir;
				if (current_idx + 1 < state_refs.rows()) {
					path_dir = {state_refs(current_idx + 1, 0) - x, state_refs(current_idx + 1, 1) - y};
				} else if (current_idx - 1 >= 0) {
					path_dir = {x - state_refs(current_idx - 1, 0), y - state_refs(current_idx - 1, 1)};
				} else {
					current_idx += static_cast<int>(density * threshold * 0.33);
					continue;
				}

				if (vec_to_intersection.dot(path_dir) > 0) {
					intersection_indices.push_back(closest_idx);
					current_idx += static_cast<int>(density * threshold * 0.9);
					continue;
				}
			}
		}

		current_idx += static_cast<int>(density * threshold * 0.33);
	}

	if (!intersection_indices.empty()) {
		auto stop = std::chrono::high_resolution_clock::now();
		auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
		utils.debug("FIND_INTERSECTIONS(): SUCCESS: found " + std::to_string(intersection_indices.size()) + " intersections. time: " + std::to_string(duration.count()) + "us", 1);

		for (int i = 0; i < intersection_indices.size(); ++i) {
			const auto &inter = intersections_all[intersection_indices[i]].pose;
			std::string associated_sign_type = "nullptr";
			if (intersections_all[intersection_indices[i]].associated_sign != nullptr) {
				associated_sign_type = OBJECT_NAMES[intersections_all[intersection_indices[i]].associated_sign->type];
			}
			utils.debug(std::to_string(i) + ") [" + helper::d2str(inter[0]) + ", " + helper::d2str(inter[1]) + "], yaw: " + helper::d2str(inter[2]) + ", associated sign: " + associated_sign_type, 1);
		}
		return true;
	}

	utils.debug("FIND_INTERSECTIONS(): FAILURE: no intersections found", 1);
	return false;
}

inline bool attribute_cmp(int idx, int attr) {
  if (idx < 0 || idx >= state_attributes.size()) {
		return false;
	}
	return state_attributes(idx) == attr || state_attributes(idx) == attr + 100;
}

inline bool is_not_detectable(int idx) {
	return state_attributes(idx) >= 100 || attribute_cmp(idx, ATTRIBUTE::DOTTED_CROSSWALK) || attribute_cmp(idx, ATTRIBUTE::INTERSECTION) || attribute_cmp(idx, ATTRIBUTE::ROUNDABOUT);
}

inline bool lane_detectable(int start_idx, int end_idx) {
  const static std::vector<int> detectable_attributes = {ATTRIBUTE::NORMAL, ATTRIBUTE::ONEWAY, ATTRIBUTE::DOTTED, ATTRIBUTE::CROSSWALK};
	if (start_idx < 0 || start_idx >= state_attributes.size()) {
		return false;
	}
	if (end_idx >= state_attributes.size()) {
		return false;
	}
	bool start_idx_detectable = false;
	bool end_idx_detectable = false;
	for (int i = 0; i < detectable_attributes.size(); i++) {
		if (attribute_cmp(start_idx_detectable, detectable_attributes[i])) {
			start_idx_detectable = true;
		}
		if (attribute_cmp(end_idx, detectable_attributes[i])) {
			end_idx_detectable = true;
		}
	}
	return start_idx_detectable && end_idx_detectable;
}

inline void change_lane(int start_index, int end_index, bool shift_right = false, double shift_distance = 0.36 - 0.1) {
  if (shift_right)
		shift_distance *= -1;

	// Total number of points
	int total_points = end_index - start_index;
	// int ramp_length = static_cast<int>(density * VehicleConstants::CAR_LENGTH / 2);
	int ramp_length = static_cast<int>(density * 0.125);

	// Define the start and end indices of the constant shift phase
	int ramp_up_end = start_index + ramp_length;
	int ramp_down_start = end_index - ramp_length;

	// Iterate over each point from start_index to end_index
	for (int i = start_index; i < end_index; i++) {
		double current_shift = 0.0;

		// Ramp up phase
		if (i < ramp_up_end) {
			// Adjust the progress calculation to start shifting immediately after start_index
			double progress = static_cast<double>(i - start_index + 1) / ramp_length;
			current_shift = shift_distance * progress;
		}
		// Constant shift phase
		else if (i >= ramp_up_end && i < ramp_down_start) {
			current_shift = shift_distance;
		}
		// Ramp down phase
		else {
			// Adjust progress calculation for a smoother transition to zero at the end
			double progress = static_cast<double>(end_index - i) / ramp_length;
			current_shift = shift_distance * progress;
		}

		state_refs.block(i, 0, 1, 2) += normals.block(i, 0, 1, 2) * current_shift;
	}
}

inline int get_current_attribute() {
    return state_attributes(target_waypoint_index);
}

inline void get_current_waypoints(Eigen::MatrixXd& output) {
  	int start = std::min(target_waypoint_index, static_cast<int>(state_refs.rows()) - 2);
		int end = std::min(N, static_cast<int>(state_refs.rows()) - target_waypoint_index - 1);
		end = std::max(end, 1);
		output = state_refs.block(start, 0, end, 3);
}

inline int find_closest_waypoint2(
	const Eigen::Vector2d& pt,
	double threshold = 0.1)
{
	const int M = state_refs.rows();
	if (M == 0) return -1;

	// clamp our search window
	int min_index = 0;
	int max_index = M - 1;
	const int window_size = max_index - min_index + 1;
	if (window_size <= 0) return -1;

	// 1) grab the x,y columns and slice out [min_index..max_index]
	//    gives a (window_size x 2) matrix
	auto xy_win = state_refs
									.leftCols<2>()
									.middleRows(min_index, window_size);

	// 2) subtract pt from every row (broadcast)
	//    diff(i,0:1) = xy_win(i,0:1) - pt
	Eigen::MatrixXd diff = xy_win.rowwise() - pt.transpose();

	// 3) squared norms of each row -> VectorXd of length window_size
	Eigen::VectorXd dist2 = diff.rowwise().squaredNorm();

	// build an index‐vector [min_index, min_index+1, …, max_index]
	Eigen::VectorXi idxs = Eigen::VectorXi::LinSpaced(window_size, min_index, max_index);

	// mask: if dist2(i) < threshold^2, keep idxs(i), else -1
	double thr2 = threshold * threshold;
	Eigen::VectorXi valid = (dist2.array() < thr2)
													 .select(idxs, Eigen::VectorXi::Constant(window_size, -1));

	// the answer is the maximum entry of valid (or –1 if all were –1)
	int best = valid.maxCoeff();
	return best;
}

inline int find_closest_waypoint(const Eigen::Vector3d& x_current, int min_index = -1, int max_index = -1) {
	double current_norm = x_current.head(2).squaredNorm();

	double min_distance_sq = std::numeric_limits<double>::max();
	int closest = -1;

	static int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]

	if (min_index < 0)
		min_index = std::min(last_waypoint_index, static_cast<int>(state_refs.rows()) - 1);
	if (max_index < 0)
		max_index = std::min(target_waypoint_index + limit, static_cast<int>(state_refs.rows()) - 1); // state_refs.rows() - 1;

	for (int i = max_index; i >= min_index; --i) {
		double distance_sq = (state_refs.row(i).head(2).squaredNorm() - 2 * state_refs.row(i).head(2).dot(x_current.head(2)) + current_norm);

		if (distance_sq < min_distance_sq) {
			min_distance_sq = distance_sq;
			closest = i;
		}
	}
	closest_waypoint_index = closest;
	return closest;
}

inline int find_next_waypoint(int& output_target, const Eigen::Vector3d& i_current_state, int min_index = -1, int max_index = -1) {
  	int target = 0;
		static int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]
		static int lookahead = 1;
		if (v_ref > 0.375) lookahead = 1;

		static int count = 0;
		closest_waypoint_index = find_closest_waypoint(i_current_state, min_index, max_index);
		double distance_to_current = std::sqrt((state_refs(closest_waypoint_index, 0) - i_current_state[0]) * (state_refs(closest_waypoint_index, 0) - i_current_state[0]) +
											   (state_refs(closest_waypoint_index, 1) - i_current_state[1]) * (state_refs(closest_waypoint_index, 1) - i_current_state[1]));
		if (distance_to_current > 1.2) {
			std::cout << "WARNING: PathManager::find_next_waypoint(): distance to closest waypoint is too large: " << distance_to_current << std::endl;
			min_index = static_cast<int>(std::max(closest_waypoint_index - distance_to_current * density * 1.2, 0.0));
			closest_waypoint_index = find_closest_waypoint(i_current_state, min_index, max_index);
		}

		if (count >= 8) {
			target = closest_waypoint_index + lookahead;
			count = 0;
		} else {
			target = target_waypoint_index + 1;
			count++;
		}

		output_target = std::min(target, static_cast<int>((*state_refs_ptr).rows()) - 1);
		last_waypoint_index = output_target;
		return 1;
}

inline void reset_target_waypoint_index(const Eigen::Vector3d& x_current) {
    target_waypoint_index = find_closest_waypoint(x_current);
}

inline bool is_straight_line(int start_idx, int num_waypoints, double target_angle, double threshold) {
  	int N = state_refs.rows();

		// Ensure the start index and number of waypoints are within valid bounds
		if (start_idx < 0 || start_idx >= N || num_waypoints <= 0 || start_idx + num_waypoints > N) {
			std::cerr << "Invalid index range." << std::endl;
			return false;
		}

		target_angle = helper::yaw_mod(target_angle); // now between -pi and pi
		for (int i = start_idx; i < start_idx + num_waypoints; ++i) {
			double yaw = state_refs(i, 2);
			yaw = helper::yaw_mod(yaw); // between -pi and pi
			double diff = helper::compare_yaw(target_angle, yaw);
			// Check if yaw is within the threshold of target_angle
			if (diff > threshold) {
				return false;
			}
		}
		return true;
}

inline void remove_large_yaw_jump() {
	for (int i = 2; i < state_refs.rows(); i++) {
		double diff = state_refs(i, 2) - state_refs(i - 1, 2);
		if (std::abs(diff) > 1 && std::abs(diff) < 5) {
			state_refs(i, 2) = 2 * state_refs(i - 1, 2) - state_refs(i - 2, 2);
		}
	}
	for (int i = 1; i < state_refs.rows(); i++) {
		double diff = state_refs(i, 2) - state_refs(i - 1, 2);
		while (diff > M_PI) {
			state_refs(i, 2) -= 2 * M_PI;
			diff = state_refs(i, 2) - state_refs(i - 1, 2);
		}
		while (diff < -M_PI) {
			state_refs(i, 2) += 2 * M_PI;
			diff = state_refs(i, 2) - state_refs(i - 1, 2);
		}
	}
}

inline bool set_params(const std::shared_ptr<TcpClient>& tcp_client) {
	std::vector<double> state_refs_v(state_refs.data(), state_refs.data() + state_refs.size());
	nh->setParam("/state_refs", state_refs_v);
	std::vector<double> state_attributes_v(state_attributes.data(), state_attributes.data() + state_attributes.size());
	nh->setParam("/state_attributes", state_attributes_v);
	std_srvs::Trigger trigger_srv;

	tcp_client->send_trigger(trigger_srv);
	tcp_client->send_params(state_refs_v, state_attributes_v);

	bool success = false;
	size_t retries = 50;
	size_t try_count = 0;
	std::optional<std_srvs::TriggerResponse> response;

	while (try_count < retries) {
		if (tcp_client->get_trigger_msgs().size() > 0) {
			response = tcp_client->get_trigger_msgs().front()->response;
			tcp_client->get_trigger_msgs().pop();
			success = true;
			std::cout << "ROS node set params notification ack = success." << std::endl;
			break;
		}
		try_count++;
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
	}

	if (success) {
		if (response.value().success) {
			ROS_INFO("Python node notified successfully.");
		} else {
			ROS_WARN("Python node notification failed: %s", trigger_srv.response.message.c_str());
		}
	} else {
		ROS_ERROR("Failed to call the notification service.");
	}

	return true;
}

inline bool call_waypoint_service(double x, double y, double yaw, const std::shared_ptr<TcpClient>& tcp_client) {
	std_msgs::Float32MultiArray state_refs_in;
	std_msgs::Float32MultiArray input_refs_in;
	std_msgs::Float32MultiArray wp_attributes_in;
	std_msgs::Float32MultiArray wp_normals_in;
	path_planner.set_constraints(v_ref, N, T, x, y, pathName);
	path_planner.plan_path(state_refs_in, input_refs_in, wp_attributes_in, wp_normals_in);

	std::vector<double> state_refs_v(state_refs_in.data.begin(), state_refs_in.data.end());			 // N by 3
	std::vector<double> input_refs_v(input_refs_in.data.begin(), input_refs_in.data.end());			 // N by 2
	std::vector<double> wp_attributes_v(wp_attributes_in.data.begin(), wp_attributes_in.data.end()); // N by 1
	std::vector<double> wp_normals_v(wp_normals_in.data.begin(), wp_normals_in.data.end());			 // N by 2
	int N = state_refs_v.size() / 3;
	state_refs = Eigen::Map<Eigen::MatrixXd>(state_refs_v.data(), 3, N).transpose();
	remove_large_yaw_jump();
	input_refs = Eigen::Map<Eigen::MatrixXd>(input_refs_v.data(), 2, N).transpose();
	state_attributes = Eigen::Map<Eigen::VectorXd>(wp_attributes_v.data(), N);
	normals = Eigen::Map<Eigen::MatrixXd>(wp_normals_v.data(), 2, N).transpose();

	ROS_INFO("initialize(): Received waypoints of size %d", N);
	tcp_client->send_waypoints_srv(state_refs_in, input_refs_in, wp_attributes_in, wp_normals_in);
	set_params(tcp_client);
	return true;
}

inline bool call_go_to_service(double x, double y, double yaw, double dest_x, double dest_y) {
	utils::go_to srv;
	srv.request.x0 = x;
	srv.request.y0 = y;
	srv.request.yaw0 = yaw;
	srv.request.dest_x = dest_x;
	srv.request.dest_y = dest_y;
	// convert v_ref to string
	int vrefInt;
	if (!nh->getParam("/vrefInt", vrefInt)) {
		ROS_ERROR("Failed to get param 'vrefInt'");
		vrefInt = 25;
	}
	srv.request.vrefName = std::to_string(vrefInt);
	if (go_to_client.waitForExistence(ros::Duration(5))) {
		ROS_INFO("go_to service found");
	} else {
		ROS_INFO("go_to service not found after 5 seconds");
		return false;
	}
	if (go_to_client.call(srv)) {
		std::vector<double> state_refs_v(srv.response.state_refs.data.begin(), srv.response.state_refs.data.end());			 // N by 3
		std::vector<double> input_refs_v(srv.response.input_refs.data.begin(), srv.response.input_refs.data.end());			 // N by 2
		std::vector<double> wp_attributes_v(srv.response.wp_attributes.data.begin(), srv.response.wp_attributes.data.end()); // N by 1
		std::vector<double> wp_normals_v(srv.response.wp_normals.data.begin(), srv.response.wp_normals.data.end());			 // N by 2
		int N = state_refs_v.size() / 3;
		state_refs = Eigen::Map<Eigen::MatrixXd>(state_refs_v.data(), 3, N).transpose();
		remove_large_yaw_jump();
		input_refs = Eigen::Map<Eigen::MatrixXd>(input_refs_v.data(), 2, N).transpose();
		state_attributes = Eigen::Map<Eigen::VectorXd>(wp_attributes_v.data(), N);
		normals = Eigen::Map<Eigen::MatrixXd>(wp_normals_v.data(), 2, N).transpose();

		ROS_INFO("initialize(): Received waypoints of size %d", N);
		target_waypoint_index = 0;
		last_waypoint_index = target_waypoint_index;
		closest_waypoint_index = 0;
		return true;
	} else {
		ROS_INFO("ERROR: initialize(): Failed to call service waypoints");
		return false;
	}
}

inline bool call_go_to_multiple_service(double x, double y, double yaw, std::vector<std::tuple<float, float>>& destinations) {
	std_msgs::Float32MultiArray state_refs_in;
	std_msgs::Float32MultiArray input_refs_in;
	std_msgs::Float32MultiArray wp_attributes_in;
	std_msgs::Float32MultiArray wp_normals_in;
	path_planner.set_constraints(v_ref, N, T, x, y, destinations);
	path_planner.plan_path(state_refs_in, input_refs_in, wp_attributes_in, wp_normals_in);

	std::vector<double> state_refs_v(state_refs_in.data.begin(), state_refs_in.data.end());			 // N by 3
	std::vector<double> input_refs_v(input_refs_in.data.begin(), input_refs_in.data.end());			 // N by 2
	std::vector<double> wp_attributes_v(wp_attributes_in.data.begin(), wp_attributes_in.data.end()); // N by 1
	std::vector<double> wp_normals_v(wp_normals_in.data.begin(), wp_normals_in.data.end());			 // N by 2

	int N = state_refs_v.size() / 3;
	state_refs = Eigen::Map<Eigen::MatrixXd>(state_refs_v.data(), 3, N).transpose();
	remove_large_yaw_jump();
	input_refs = Eigen::Map<Eigen::MatrixXd>(input_refs_v.data(), 2, N).transpose();
	state_attributes = Eigen::Map<Eigen::VectorXd>(wp_attributes_v.data(), N);
	normals = Eigen::Map<Eigen::MatrixXd>(wp_normals_v.data(), 2, N).transpose();

	ROS_INFO("initialize(): Received waypoints of size %d", N);
	target_waypoint_index = 0;
	last_waypoint_index = target_waypoint_index;
	closest_waypoint_index = 0;
	return true;
}

inline Eigen::MatrixXd smooth_yaw_angles(const Eigen::MatrixXd& state_refs) {
	int N = state_refs.rows();
	if (N == 0)
		return state_refs; // Return empty if no data

	Eigen::MatrixXd smoothed_refs = state_refs;

	// Extract yaw values
	Eigen::VectorXd yaw_angles = state_refs.col(2);

	// Compute differences between consecutive yaw values
	Eigen::VectorXd diffs = yaw_angles.tail(N - 1) - yaw_angles.head(N - 1);

	// Adjust large jumps in yaw angles
	for (int i = 0; i < diffs.size(); ++i) {
		if (diffs(i) > M_PI * 0.8) {
			diffs(i) -= 2 * M_PI;
		} else if (diffs(i) < -M_PI * 0.8) {
			diffs(i) += 2 * M_PI;
		}
	}

	Eigen::VectorXd smooth_yaw(N);
	smooth_yaw(0) = yaw_angles(0);
	for (int i = 1; i < N; ++i) {
		smooth_yaw(i) = smooth_yaw(i - 1) + diffs(i - 1);
	}

	smoothed_refs.col(2) = smooth_yaw;

	return smoothed_refs;
}

} // namespace PathManager
