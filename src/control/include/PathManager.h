#pragma once

#include "GroundTruth.h"
#include "PathPlanner.hpp"
#include "TcpClient.hpp"
#include "Utility.hpp"
#include "utils/helper.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/TriggerResponse.h"
#include "utils/constants.h"
#include "utils/go_to.h"
#include "utils/go_to_multiple.h"
#include "utils/waypoints.h"
#include <Eigen/Dense>
#include <chrono>
#include <cmath>
#include <ros/ros.h>
#include <std_srvs/Trigger.h>
#include <vector>
#include <string>
#include <atomic>

namespace PathManager {

inline std::atomic<bool> path_manager_initialized = false;
inline ros::NodeHandle* nh = nullptr;
inline PathPlanner path_planner{0.32, 40, 0.1};

inline double T = 0.1;
inline int N = 40;
inline double v_ref = 0.32;
inline std::string pathName = "speedrun";

inline double density = 1.0 / T / v_ref;
inline double region_of_acceptance = 0.03076923 * 3 * (0.1 * 1.3) / density;
inline double region_of_acceptance_cw = region_of_acceptance / 1.5;
inline double region_of_acceptance_hw = region_of_acceptance * 1.5;

inline double t0 = 0.0;
inline int closest_waypoint_index = 0;
inline int target_waypoint_index = 0;
inline int last_waypoint_index = 0;
inline int overtake_end_index = 0;
// inline int overtake_end_index_scaler = 1.15;
inline double rdb_circumference = 3.95;
inline bool debug = true;

inline int v_ref_int = static_cast<int>(v_ref * 100);

inline ros::ServiceClient waypoints_client;
inline ros::ServiceClient go_to_client;
inline ros::ServiceClient go_to_multiple_client;
inline ros::ServiceClient trigger_client;

inline Eigen::MatrixXd state_refs, state_refs_original;
inline Eigen::MatrixXd input_refs;
inline Eigen::MatrixXd normals;
inline Eigen::MatrixXd left_turn_states;
inline Eigen::MatrixXd right_turn_states;
inline Eigen::MatrixXd straight_states;
inline Eigen::VectorXd state_attributes;
inline Eigen::MatrixXd* state_refs_ptr = &state_refs;

inline std::vector<int> intersection_indices;
inline std::vector<int> intersection_state_refs_indices;
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
	intersection_state_refs_indices.clear();
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
					intersection_state_refs_indices.push_back(current_idx);
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
			utils.debug(std::to_string(i) + ") [" + helper::d2str(inter[0]) + ", " + helper::d2str(inter[1]) + "], yaw: " + helper::d2str(inter[2]) + ", associated sign: " + associated_sign_type + ", stateref index: " + std::to_string(intersection_state_refs_indices[i]), 1);
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

inline bool attribute_cmp2(int idx, int idx2) {
  if (idx < 0 || idx >= state_attributes.size()) {
		return false;
	}
	if (idx2 < 0 || idx2 >= state_attributes.size()) {
		return false;
	}
	return state_attributes(idx) == state_attributes(idx2);
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

inline int change_lane(int start_idx, int end_idx, bool shift_right = false, double shift_distance = 0.36 - 0.1) {
	if (shift_right) shift_distance *= -1.0;

    // 1) Precompute for originals: positions, normals, ramp‐factors (t_i), and yaw
    int N = end_idx - start_idx;
    if (N < 2) return 0;

    std::vector<Eigen::Vector2d>  orig_pos(N);
    std::vector<Eigen::Vector2d>  orig_nrm(N);
    std::vector<double>           ramp_t(N);
    std::vector<double>           orig_yaw(N);

    int ramp_len       = static_cast<int>(density * 0.125);
    int ramp_up_end    = ramp_len;
    int ramp_down_start= N - ramp_len;

    for (int i = 0; i < N; ++i) {
        // original
        auto row = state_refs.row(start_idx + i);
        orig_pos[i] = row.head<2>();
        orig_yaw[i] = row(2);

        // normal at that original
        orig_nrm[i].x() = -std::sin(orig_yaw[i]);
        orig_nrm[i].y() =  std::cos(orig_yaw[i]);

        // ramp factor t_i ∈ [0..1]
        if      (i < ramp_up_end)     ramp_t[i] = double(i+1)/ramp_len;
        else if (i < ramp_down_start) ramp_t[i] = 1.0;
        else                          ramp_t[i] = double(N-i)/ramp_len;
    }

    // 2) Shift all originals in place
    std::vector<Eigen::Vector3d> shifted;
    shifted.reserve(N);
    for (int i = 0; i < N; ++i) {
        Eigen::Vector2d p = orig_pos[i]
                         + orig_nrm[i] * (shift_distance * ramp_t[i]);
        shifted.emplace_back(p.x(), p.y(), orig_yaw[i]);
    }

    // 3) Densify + fill gaps by interpolating pos, normal & ramp_t
    //    so every new point is still shifted exactly perpendicular.
    std::vector<Eigen::Vector3d> densified;
    densified.reserve(N * 2);

    auto lerp2 = [&](auto &A, auto &B, double u){ return A + u*(B-A); };

    // compute average spacing from originals
    double total_len = 0;
    for (int i = 1; i < N; ++i)
        total_len += (orig_pos[i] - orig_pos[i-1]).norm();
    double avg_sp = total_len / (N-1);

		double densify_factor = 0.25;
    for (int i = 0; i+1 < N; ++i) {
        // always push the already‐shifted original point
        densified.push_back(shifted[i]);

        // how many to insert?
        double gap = (shifted[i+1].head<2>() - shifted[i].head<2>()).norm();
				double desired_spacing = avg_sp / densify_factor;
        int nins = std::max(0, int(std::round(gap/desired_spacing)) - 1);

        for (int k = 1; k <= nins; ++k) {
            double u = double(k)/(nins+1);

            // 3a) interpolate the centerline position
            Eigen::Vector2d center = lerp2(orig_pos[i], orig_pos[i+1], u);

            // 3b) interpolate & renormalize the normal
            Eigen::Vector2d n_interp = (1-u)*orig_nrm[i] + u*orig_nrm[i+1];
            n_interp.normalize();

            // 3c) interpolate the ramp factor
            double t_interp = (1-u)*ramp_t[i] + u*ramp_t[i+1];

            // 3d) apply shift exactly perpendicular
            Eigen::Vector2d p_mid = center + n_interp * (shift_distance * t_interp);

            // 3e) yaw will be fixed later—set placeholder
            densified.emplace_back(p_mid.x(), p_mid.y(), 0.0);
        }
    }
    // finally push the last original
    densified.push_back(shifted.back());

    // 4) Recompute & unwrap yaws exactly as before:
    densified[0].z() = shifted[0].z();
    for (int i = 1; i < (int)densified.size(); ++i) {
        double raw = std::atan2(
            densified[i].y() - densified[i-1].y(),
            densified[i].x() - densified[i-1].x()
        );
        double prev = densified[i-1].z();
        double d = std::remainder(raw - prev, 2.0*M_PI);
        densified[i].z() = prev + d;
    }

    // 5) Splice back into state_refs and return extra count
    int extra = int(densified.size()) - N;
    int old_n = state_refs.rows();
    int new_n = old_n + extra;
    Eigen::MatrixXd M(new_n, 3);

    // copy before
    if (start_idx > 0)
        M.block(0,0,start_idx,3) = state_refs.block(0,0,start_idx,3);

    // copy densified
    for (int i = 0; i < (int)densified.size(); ++i)
        M.row(start_idx + i) = densified[i].transpose();

    // copy after
    int tail = old_n - end_idx;
    if (tail > 0)
        M.block(start_idx + densified.size(),0,tail,3)
         = state_refs.block(end_idx,0,tail,3);

    state_refs.swap(M);

		// Eigen::MatrixXd normals_new(new_n, 2);
    // if (start_idx > 0)
    //     normals_new.block(0, 0, start_idx, 2) = normals.block(0, 0, start_idx, 2);
    // for (int i = 0; i < (int)densified_nrm.size(); ++i)
    //     normals_new.row(start_idx + i) = densified_nrm[i].transpose();
    // int tail = old_n - end_idx;
    // if (tail > 0)
    //     normals_new.block(start_idx + densified_nrm.size(), 0, tail, 2)
    //         = normals.block(end_idx, 0, tail, 2);
    // normals.swap(normals_new);

    Eigen::MatrixXd inputs_new(new_n, input_refs.cols());
		if (start_idx > 0) {
				inputs_new.block(0, 0, start_idx, input_refs.cols())
						= input_refs.block(0, 0, start_idx, input_refs.cols());
		}
		for (int i = 0; i < (int)densified.size(); ++i) {
				inputs_new.row(start_idx + i) = input_refs.row(start_idx);
		}
		tail = old_n - end_idx;
		if (tail > 0) {
				inputs_new.block(start_idx + densified.size(), 0, tail, input_refs.cols())
						= input_refs.block(end_idx, 0, tail, input_refs.cols());
		}
		input_refs.swap(inputs_new);

    Eigen::VectorXd attrs_new(new_n);
    if (start_idx > 0)
        attrs_new.segment(0, start_idx) = state_attributes.segment(0, start_idx);
    for (int i = 0; i < (int)densified.size(); ++i) {
        attrs_new[start_idx + i] = state_attributes[start_idx];
    }
    if (tail > 0)
        attrs_new.segment(start_idx + densified.size(), tail)
            = state_attributes.segment(end_idx, tail);
    state_attributes.swap(attrs_new);

		for (size_t k = intersection_index; k < intersection_state_refs_indices.size(); ++k) {
			// these are all intersections we haven’t reached yet,
			// and by assumption theyre after the lane‐change window
			intersection_state_refs_indices[k] += extra;
		}
		return extra;
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
  double threshold = 0.1, int start_idx = -1, int end_idx = -1)
{
	if (start_idx < 0) start_idx = 0;
	if (end_idx < 0)   end_idx = state_refs_original.rows() - 1;
	end_idx = std::min(end_idx, static_cast<int>(state_refs_original.rows() - 1));
	start_idx = std::max(start_idx, 0);
	if (start_idx > end_idx) return -1;

  const double thr2 = threshold * threshold;
  int   bestIdx   = -1;
  double bestDist2 = thr2;  // only accept < thr2

	for (int i = start_idx; i <= end_idx; ++i) {
    double dx    = state_refs_original(i, 0) - pt.x();
    double dy    = state_refs_original(i, 1) - pt.y();
    double dist2 = dx*dx + dy*dy;

    if (dist2 < bestDist2) {
      bestDist2 = dist2;
      bestIdx   = i;
    }
  }
  return bestIdx;
}

inline int find_closest_waypoint(const Eigen::Vector3d& x_current, int min_index = -1, int max_index = -1) {
	double current_norm = x_current.head(2).squaredNorm();

	double min_distance_sq = std::numeric_limits<double>::max();
	int closest = -1;

	int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]
	if (min_index < 0)
		min_index = std::min(last_waypoint_index, static_cast<int>(state_refs.rows()) - 1);
	if (max_index < 0)
		max_index = std::min(target_waypoint_index + limit, static_cast<int>(state_refs.rows()) - 1); // state_refs.rows() - 1;

	// std::cout << "rdb circumference: " << rdb_circumference << ", v_ref: " << v_ref << ", T: " << T << ", limit: " << limit << ", target_waypoint_index: " << target_waypoint_index << ", min_index: " << min_index << ", max_index: " << max_index << ", stateref rows: " << state_refs.rows() << std::endl;
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

inline int find_next_waypoint_count = 5;
inline int find_next_waypoint(int& output_target, const Eigen::Vector3d& i_current_state, int min_index = -1, int max_index = -1) {
  	int target = 0;
		int lookahead = 1;
		if (v_ref > 0.375) lookahead = 1;

		closest_waypoint_index = find_closest_waypoint(i_current_state, min_index, max_index);
		double distance_to_current = std::sqrt((state_refs(closest_waypoint_index, 0) - i_current_state[0]) * (state_refs(closest_waypoint_index, 0) - i_current_state[0]) +
											   (state_refs(closest_waypoint_index, 1) - i_current_state[1]) * (state_refs(closest_waypoint_index, 1) - i_current_state[1]));
		if (distance_to_current > 1.2) {
			std::cout << "WARNING: PathManager::find_next_waypoint(): distance to closest waypoint is too large: " << distance_to_current << std::endl;
			min_index = static_cast<int>(std::max(closest_waypoint_index - distance_to_current * density * 1.2, 0.0));
			closest_waypoint_index = find_closest_waypoint(i_current_state, min_index, max_index);
		}

		std::cout << "PathManager:: find_next_waypoint_count = " << find_next_waypoint_count << std::endl;
		if (find_next_waypoint_count >= 8) {
			target = closest_waypoint_index + lookahead;
			// std::cout << "PathManager::find_next_waypoint(): count >= 8, setting target to closest waypoint index + lookahead: " << target << std::endl;
			find_next_waypoint_count = 0;
		} else {
			// std::cout << "PathManager:: incrementing find_next_waypoint_count to " << find_next_waypoint_count + 1 << std::endl;
			target = target_waypoint_index + 1;
			find_next_waypoint_count++;
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
			// std::cerr << "Invalid index range." << std::endl;
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

    tcp_client->set_trigger_response_callback(
        [](const std_srvs::TriggerResponse &resp) {
            if (resp.success) {
                ROS_INFO("Python node notified successfully.");
            } else {
                ROS_WARN("Python node notification failed: %s", resp.message.c_str());
            }
        }
    );

	std_srvs::Trigger trigger_srv;
	tcp_client->send_trigger(trigger_srv);
	tcp_client->send_params(state_refs_v, state_attributes_v);
    return true;
}

inline bool call_waypoint_service(double x, double y, double yaw, const std::shared_ptr<TcpClient>& tcp_client) {
	std_msgs::Float32MultiArray state_refs_in;
	std_msgs::Float32MultiArray input_refs_in;
	std_msgs::Float32MultiArray wp_attributes_in;
	std_msgs::Float32MultiArray wp_normals_in;
	path_planner.set_constraints(v_ref, N, T, x, y, pathName, Tunable::useGps);
	path_planner.plan_path(state_refs_in, input_refs_in, wp_attributes_in, wp_normals_in);

	std::vector<double> state_refs_v(state_refs_in.data.begin(), state_refs_in.data.end());			 // N by 3
	std::vector<double> input_refs_v(input_refs_in.data.begin(), input_refs_in.data.end());			 // N by 2
	std::vector<double> wp_attributes_v(wp_attributes_in.data.begin(), wp_attributes_in.data.end()); // N by 1
	std::vector<double> wp_normals_v(wp_normals_in.data.begin(), wp_normals_in.data.end());			 // N by 2
	int N = state_refs_v.size() / 3;
	state_refs = Eigen::Map<Eigen::MatrixXd>(state_refs_v.data(), 3, N).transpose();
	remove_large_yaw_jump();
	state_refs_original = state_refs;
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
		state_refs_original = state_refs;
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

inline bool call_go_to_multiple_service(double x, double y, double yaw, const std::vector<std::tuple<float, float>>& destinations) {
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
	state_refs_original = state_refs;
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
