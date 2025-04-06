#ifndef PathManager_HPP
#define PathManager_HPP

#include <ros/ros.h>
#include <thread>
#include <vector>
#include <string>
#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <limits.h>
#include <cmath>
#include "TcpClient.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_srvs/TriggerResponse.h"
#include "utils/Point2D.h"
#include "utils/constants.h"
#include "utils/goto_command.h"
#include "utils/waypoints.h"
#include "utils/go_to.h"
#include "utils/go_to_multiple.h"
#include <std_srvs/Trigger.h>
#include "utility.hpp"
#include "utils/helper.h"

class PathManager {
public:
    PathManager(ros::NodeHandle& nh_, double T, int N, double v_ref):
        nh(nh), T(T), N(N), v_ref(v_ref), density(1/T/v_ref), region_of_acceptance(0.03076923*3 * (0.125*1.3) / density), 
        region_of_acceptance_cw(region_of_acceptance * 1.0/1.5), region_of_acceptance_hw(region_of_acceptance * 1.5), t0(0.0), closest_waypoint_index(0)
    {
        std::cout << "Path Manager Constructor" << std::endl;
        v_ref_int = static_cast<int>(v_ref * 100); // convert to cm/s
        
        // std::string dir = helper::getSourceDirectory();
        // std::string v_ref_int_str = std::to_string(v_ref_int);
        // std::string path_name = "_speedrun";
        // path_name = "_path1";
        // state_refs = loadTxt(dir + "/../../../control/scripts/paths/state_refs" + path_name +v_ref_int_str+ ".txt");
        // remove_large_yaw_jump();
        // input_refs = loadTxt(dir + "/../../../control/scripts/paths/input_refs" + path_name +v_ref_int_str+ ".txt");
        // state_attributes = loadTxt(dir + "/../../../control/scripts/paths/wp_attributes" + path_name +v_ref_int_str+ ".txt");
        // normals = loadTxt(dir + "/../../../control/scripts/paths/wp_normals"+ path_name +v_ref_int_str+ ".txt");

        state_refs_ptr = &state_refs;
        target_waypoint_index = 0;
        last_waypoint_index = target_waypoint_index;

        std::cout << "v ref: " << v_ref << ", int:" << v_ref_int << std::endl;

        waypoints_client = nh.serviceClient<utils::waypoints>("/waypoint_path");
        if(!nh.getParam("/pathName", pathName)) {
            ROS_ERROR("Failed to get param 'pathName'");
            pathName = "speedrun";
        }

        go_to_client = nh.serviceClient<utils::go_to>("/go_to");
        go_to_multiple_client = nh.serviceClient<utils::go_to_multiple>("/go_to_multiple");
        trigger_client = nh.serviceClient<std_srvs::Trigger>("/notify_params_updated");
    }

    PathManager(ros::NodeHandle& nh_): PathManager(nh_, 0.125, 40, 0.25) {}
    ~PathManager() {}

    ros::NodeHandle nh;
    ros::ServiceClient waypoints_client;
    ros::ServiceClient go_to_client;
    ros::ServiceClient go_to_multiple_client;
    ros::ServiceClient trigger_client;
    std::string pathName;
    int target_waypoint_index=0, last_waypoint_index=0, closest_waypoint_index=0;
    int overtake_end_index = 0;
    int overtake_end_index_scaler = 1.15;
    int v_ref_int;
    int N;
    double region_of_acceptance, region_of_acceptance_cw, region_of_acceptance_hw, v_ref, t0, T, density, rdb_circumference = 3.95;
    bool debug = true;
    Eigen::MatrixXd state_refs, input_refs, normals, left_turn_states, right_turn_states, straight_states;
    Eigen::MatrixXd *state_refs_ptr;
    Eigen::VectorXd state_attributes;
    std::vector<Eigen::Vector3d> intersection_points;
    int intersection_index = 0;

    bool find_intersections(Utility& utils) {
        utils.debug("FIND_INTERSECTIONS(): started", 1);
        auto start = std::chrono::high_resolution_clock::now();
        intersection_points.clear();
        intersection_index = 0;
        if (state_refs.rows() == 0) {
            utils.debug("FIND_INTERSECTIONS(): FAILURE: state_refs is empty", 1);
            return false;
        }
        int current_idx = 0;
        int start_idx = current_idx;
        int limit = std::max(current_idx + static_cast<int>(25.0 * density), static_cast<int>(state_refs.rows() - 1));
        double threshold = INTERSECTION_DISTANCE_THRESHOLD;
        bool outer_done = false;
        while(current_idx < state_refs.rows() && !outer_done) {
            double x = state_refs(current_idx, 0);
            double y = state_refs(current_idx, 1);
            double yaw = state_refs(current_idx, 2);
            int nearest_direction_idx = Utility::nearest_direction_index(yaw);
            double nearest_direction = Utility::nearest_direction(yaw);
            double yaw_error = Utility::compare_yaw(yaw, nearest_direction);
            if (yaw_error * 180 / M_PI > 15) {
                current_idx += static_cast<int>(density * INTERSECTION_DISTANCE_THRESHOLD * 0.33 / 2);
                continue;
            }
            std::string direction = (nearest_direction_idx == 0) ? "east" :
                            (nearest_direction_idx == 1) ? "north" :
                            (nearest_direction_idx == 2) ? "west" :
                                                        "south";
            auto& INTERSECTIONS = (nearest_direction_idx == 0) ? EAST_FACING_INTERSECTIONS :
                                    (nearest_direction_idx == 1) ? NORTH_FACING_INTERSECTIONS :
                                    (nearest_direction_idx == 2) ? WEST_FACING_INTERSECTIONS :
                                                                SOUTH_FACING_INTERSECTIONS;
            double min_error_sq = std::numeric_limits<double>::max();
            int min_index = 0;
            for (size_t i = 0; i < INTERSECTIONS.size(); ++i) {
                double error_sq = std::pow(x - INTERSECTIONS[i][0], 2) + std::pow(y - INTERSECTIONS[i][1], 2);
                if (error_sq < min_error_sq) {
                    min_error_sq = error_sq;
                    min_index = static_cast<int>(i);
                }
            }
            if (min_error_sq < threshold * threshold) {
                Eigen::Vector3d intersection(INTERSECTIONS[min_index][0], INTERSECTIONS[min_index][1], INTERSECTIONS[min_index][2]);
                bool same_as_last = true;
                if (intersection_points.size() == 0) {
                    same_as_last = false;
                } else {
                    // check if it's the same as last element in intersection_points
                    double dist_to_last_sq = (intersection.head(2) - intersection_points.back().head(2)).squaredNorm();
                    same_as_last = dist_to_last_sq <= INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD * 0.75 * 0.75;
                }
                if (!same_as_last) {
                    // Ensure the intersection is in the forward direction
                    // 2D vector from the current state to the intersection.
                    Eigen::Vector2d currentPoint(x, y);
                    Eigen::Vector2d intersectionPoint(intersection[0], intersection[1]);
                    Eigen::Vector2d vecToIntersection = intersectionPoint - currentPoint;

                    // Determine the path direction vector: if possible, use the next state.
                    Eigen::Vector2d pathDirection;
                    if (current_idx + 1 < state_refs.rows()) {
                        pathDirection = Eigen::Vector2d(state_refs(current_idx + 1, 0) - x,
                                                        state_refs(current_idx + 1, 1) - y);
                    } else if (current_idx - 1 >= 0) {
                        // Fall back: use the vector from the previous state to the current one.
                        pathDirection = Eigen::Vector2d(x - state_refs(current_idx - 1, 0),
                                                        y - state_refs(current_idx - 1, 1));
                    } else {
                        // Cannot determine a path direction. Skip adding this intersection.
                        current_idx += static_cast<int>(density * INTERSECTION_DISTANCE_THRESHOLD * 0.33);
                        continue;
                    }

                    // Check if the intersection is forward: Dot product > 0 means the intersection is in the forward half-plane.
                    if (vecToIntersection.dot(pathDirection) <= 0) {
                        // Intersection is behind the current point; skip adding.
                        current_idx += static_cast<int>(density * INTERSECTION_DISTANCE_THRESHOLD * 0.33);
                        continue;
                    }
                    // std::cout << "currentPoint: " << currentPoint.transpose() << ", intersectionPoint: " << intersectionPoint.transpose() << ", nextPoint: " << Eigen::Vector2d(state_refs(current_idx + 1, 0), state_refs(current_idx + 1, 1)).transpose() << std::endl;
                    // std::cout << "vecToIntersection: " << vecToIntersection.transpose() << ", pathDirection: " << pathDirection.transpose() << ", dot: " << vecToIntersection.dot(pathDirection) << std::endl;
                    intersection_points.push_back(intersection);
                    current_idx += static_cast<int>(density * INTERSECTION_DISTANCE_THRESHOLD * 0.9);
                    continue;
                }
            } else {
                // utils.debug("FIND_CLOSEST_INTERSECTION(): FAILURE: state: (" + helper::d2str(x) + ", " + helper::d2str(y) + "), closest intersection: (" + helper::d2str(INTERSECTIONS[min_index][0]) + ", " + helper::d2str(INTERSECTIONS[min_index][1]) + "), error norm: " + helper::d2str(std::sqrt(min_error_sq)) + ", threshold: " + helper::d2str(threshold) + ", direction: " + direction + ", yaw: " + helper::d2str(yaw), 1);
            }
            current_idx += static_cast<int>(density * INTERSECTION_DISTANCE_THRESHOLD * 0.33);
        }
        if (intersection_points.size() > 0) {
            auto stop = std::chrono::high_resolution_clock::now();
            auto duration = std::chrono::duration_cast<std::chrono::microseconds>(stop - start);
            utils.debug("FIND_INTERSECTIONS(): SUCCESS: found " + std::to_string(intersection_points.size()) + " intersections. time: " + std::to_string(duration.count()) + " microseconds", 1);
            for (int i = 0; i < intersection_points.size(); i++) {
                utils.debug(std::to_string(i) + ") [" + helper::d2str(intersection_points[i][0]) + ", " + helper::d2str(intersection_points[i][1]) + "], yaw: " + helper::d2str(intersection_points[i][2]), 1);
            }
            return true;
        } else {
            utils.debug("FIND_INTERSECTIONS(): FAILURE: no intersections found", 1);
            return false;
        }
    }

    enum ATTRIBUTE {
        NORMAL, CROSSWALK, INTERSECTION, ONEWAY, HIGHWAYLEFT, HIGHWAYRIGHT, ROUNDABOUT, STOPLINE, DOTTED, DOTTED_CROSSWALK
    };
    bool attribute_cmp(int idx, int attr) {
        if (idx < 0 || idx >= state_attributes.size()) {
            return false;
        }
        return state_attributes(idx) == attr || state_attributes(idx) == attr + 100;
    }
    bool is_not_detectable(int idx) {
        return state_attributes(idx) >= 100 || attribute_cmp(idx, ATTRIBUTE::DOTTED_CROSSWALK) || attribute_cmp(idx, ATTRIBUTE::INTERSECTION) || attribute_cmp(idx, ATTRIBUTE::ROUNDABOUT);
    }
    bool lane_detectable(int start_idx, int end_idx) {
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
    void change_lane(int start_index, int end_index, bool shift_right = false, double shift_distance = 0.36-0.1) {
        if (shift_right) shift_distance *= -1;
        // state_refs.block(start_index, 0, end_index-start_index, 2) += normals.block(start_index, 0, end_index-start_index, 2) * shift_distance;

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

            // Apply the shift to the current waypoint
            state_refs.block(i, 0, 1, 2) += normals.block(i, 0, 1, 2) * current_shift;
        }
    }
    
    int get_current_attribute() {
        return state_attributes(target_waypoint_index);
    }
    
    void get_current_waypoints(Eigen::MatrixXd& output) {
        int start = std::min(target_waypoint_index, static_cast<int>(state_refs.rows()) - 2);
        int end = std::min(N, static_cast<int>(state_refs.rows()) - target_waypoint_index - 1);
        end = std::max(end, 1);
        output = state_refs.block(start, 0, end, 3);
    }
    
    // int find_next_waypoint(int &output_target, const Eigen::Vector3d &i_current_state, int min_index = -1, int max_index = -1) {
    //     int target = 0;
    //     int closest_index = find_closest_waypoint(i_current_state, min_index, max_index);
        
    //     // If no valid waypoint was found, default to index 0.
    //     if (closest_index < 0) {
    //         target = 0;
    //     } else {
    //         Eigen::Vector2d current_pos = i_current_state.head(2);
    //         Eigen::Vector2d waypoint_pos = (*state_refs_ptr).row(closest_index).head(2);
    //         double yaw = i_current_state(2);
    //         Eigen::Vector2d heading(cos(yaw), sin(yaw));
            
    //         // Compute the vector from the current position to the waypoint.
    //         Eigen::Vector2d vec_to_waypoint = waypoint_pos - current_pos;
            
    //         // If the dot product is negative, the waypoint is behind the vehicle,
    //         // so we use the next waypoint.
    //         if (heading.dot(vec_to_waypoint) < 0.0) {
    //             target = std::min(closest_index + 1, static_cast<int>((*state_refs_ptr).rows()) - 1);
    //         } else {
    //             target = closest_index;
    //         }
    //     }
        
    //     output_target = std::min(target, static_cast<int>((*state_refs_ptr).rows()) - 1);
    //     last_waypoint_index = output_target;
    //     return 1;
    // }
    int find_next_waypoint(int &output_target, const Eigen::Vector3d &i_current_state, int min_index = -1, int max_index = -1) {
        int target = 0;
        static int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]
        static int lookahead = 1;
        if (v_ref > 0.375) lookahead = 1;
        
        // static Eigen::Vector3d last_state = i_current_state;
        // double distance_travelled_sq = (i_current_state.head(2) - last_state.head(2)).squaredNorm();
        // last_state = i_current_state;

        static int count = 0;
        closest_waypoint_index = find_closest_waypoint(i_current_state, min_index, max_index);
        double distance_to_current = std::sqrt((state_refs(closest_waypoint_index, 0) - i_current_state[0]) * (state_refs(closest_waypoint_index, 0) - i_current_state[0]) + (state_refs(closest_waypoint_index, 1) - i_current_state[1]) * (state_refs(closest_waypoint_index, 1) - i_current_state[1]));
        if (distance_to_current > 1.2) {
            std::cout << "WARNING: PathManager::find_next_waypoint(): distance to closest waypoint is too large: " << distance_to_current << std::endl;
            min_index = static_cast<int>(std::max(closest_waypoint_index - distance_to_current * density * 1.2, 0.0));
            closest_waypoint_index = find_closest_waypoint(i_current_state, min_index , max_index);
        }

        if (count >= 8) {
            target = closest_waypoint_index + lookahead;
            count = 0;
        } else {
            target = target_waypoint_index + 1;
            count++;
        }
        std::cout << (*state_refs_ptr).row(target) << "closest: " << closest_waypoint_index << ", target: " << target << ", limit: " << limit << ", lookahead: " << lookahead << ", count: " << count << std::endl;

        output_target =  std::min(target, static_cast<int>((*state_refs_ptr).rows()) - 1);
        last_waypoint_index = output_target;
        return 1;
    }
    
    int find_closest_waypoint(const Eigen::Vector3d &x_current, int min_index = -1, int max_index = -1) {
        double current_norm = x_current.head(2).squaredNorm();

        double min_distance_sq = std::numeric_limits<double>::max();
        int closest = -1;

        static int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]

        if (min_index < 0) min_index = std::min(last_waypoint_index, static_cast<int>(state_refs.rows()) - 1);
        if (max_index < 0) max_index = std::min(target_waypoint_index + limit, static_cast<int>(state_refs.rows()) - 1); //state_refs.rows() - 1;
        
        // for (int i = min_index; i < max_index; ++i) {
        for (int i = max_index; i >= min_index ; --i) {
            double distance_sq = (state_refs.row(i).head(2).squaredNorm() 
                            - 2 * state_refs.row(i).head(2).dot(x_current.head(2))
                            + current_norm); 

            if (distance_sq < min_distance_sq) {
                min_distance_sq = distance_sq;
                closest = i;
            }
        }
        closest_waypoint_index = closest;
        return closest;
    }

    void reset_target_waypoint_index(const Eigen::Vector3d &x_current) {
        target_waypoint_index = find_closest_waypoint(x_current);
    }
    
    bool is_straight_line(int start_idx, int num_waypoints, double target_angle, double threshold) {
        int N = state_refs.rows();
        
        // Ensure the start index and number of waypoints are within valid bounds
        if (start_idx < 0 || start_idx >= N || num_waypoints <= 0 || start_idx + num_waypoints > N) {
            std::cerr << "Invalid index range." << std::endl;
            return false;
        }
        
        target_angle = Utility::yaw_mod(target_angle); // now between -pi and pi
        for (int i = start_idx; i < start_idx + num_waypoints; ++i) {
            double yaw = state_refs(i, 2);
            yaw = Utility::yaw_mod(yaw); // between -pi and pi
            double diff = Utility::compare_yaw(target_angle, yaw);
            // Check if yaw is within the threshold of target_angle
            if (diff > threshold) {
                return false;
            }
        }
        
        return true;
    }

    bool call_waypoint_service(double x, double y, double yaw, const std::shared_ptr<TcpClient>& tcp_client) {
        utils::waypoints srv;
        srv.request.pathName = pathName;
        srv.request.x0 = x;
        srv.request.y0 = y;
        srv.request.yaw0 = yaw;
        //convert v_ref to string
        int vrefInt;
        if(!nh.getParam("/vrefInt", vrefInt)) {
            ROS_ERROR("Failed to get param 'vrefInt'");
            vrefInt = 25;
        }
        srv.request.vrefName = std::to_string(vrefInt);
        if(waypoints_client.waitForExistence(ros::Duration(5))) {
            ROS_INFO("waypoints service found");
        } else {
            ROS_INFO("waypoints service not found after 5 seconds");
            return false;
        }
        if(waypoints_client.call(srv)) {
            std::vector<double> state_refs_v(srv.response.state_refs.data.begin(), srv.response.state_refs.data.end()); // N by 3
            std::vector<double> input_refs_v(srv.response.input_refs.data.begin(), srv.response.input_refs.data.end()); // N by 2
            std::vector<double> wp_attributes_v(srv.response.wp_attributes.data.begin(), srv.response.wp_attributes.data.end()); // N by 1
            std::vector<double> wp_normals_v(srv.response.wp_normals.data.begin(), srv.response.wp_normals.data.end()); // N by 2
            int N = state_refs_v.size() / 3;
            state_refs = Eigen::Map<Eigen::MatrixXd>(state_refs_v.data(), 3, N).transpose();
            remove_large_yaw_jump();
            input_refs = Eigen::Map<Eigen::MatrixXd>(input_refs_v.data(), 2, N).transpose();
            state_attributes = Eigen::Map<Eigen::VectorXd>(wp_attributes_v.data(), N);
            normals = Eigen::Map<Eigen::MatrixXd>(wp_normals_v.data(), 2, N).transpose();

            ROS_INFO("initialize(): Received waypoints of size %d", N);
            tcp_client->send_waypoints_srv(srv.response.state_refs, srv.response.input_refs, srv.response.wp_attributes, srv.response.wp_normals);
            set_params(tcp_client);
            return true;
        } else {
            ROS_INFO("ERROR: initialize(): Failed to call service waypoints");
            return false;
        }
    }
    
    bool call_go_to_service(double x, double y, double yaw, double dest_x, double dest_y) {
        utils::go_to srv;
        srv.request.x0 = x;
        srv.request.y0 = y;
        srv.request.yaw0 = yaw;
        srv.request.dest_x = dest_x;
        srv.request.dest_y = dest_y;
        //convert v_ref to string
        int vrefInt;
        if(!nh.getParam("/vrefInt", vrefInt)) {
            ROS_ERROR("Failed to get param 'vrefInt'");
            vrefInt = 25;
        }
        srv.request.vrefName = std::to_string(vrefInt);
        if(go_to_client.waitForExistence(ros::Duration(5))) {
            ROS_INFO("go_to service found");
        } else {
            ROS_INFO("go_to service not found after 5 seconds");
            return false;
        }
        if(go_to_client.call(srv)) {
            std::vector<double> state_refs_v(srv.response.state_refs.data.begin(), srv.response.state_refs.data.end()); // N by 3
            std::vector<double> input_refs_v(srv.response.input_refs.data.begin(), srv.response.input_refs.data.end()); // N by 2
            std::vector<double> wp_attributes_v(srv.response.wp_attributes.data.begin(), srv.response.wp_attributes.data.end()); // N by 1
            std::vector<double> wp_normals_v(srv.response.wp_normals.data.begin(), srv.response.wp_normals.data.end()); // N by 2
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

    bool call_go_to_multiple_service(double x, double y, double yaw, std::vector<std::tuple<float, float>> &destinations) {
        utils::go_to_multiple srv;
        srv.request.x0 = x;
        srv.request.y0 = y;
        srv.request.yaw0 = yaw;
        for (const auto& dest : destinations) {
            srv.request.destinations.push_back(tuple_to_point(dest));
        }
        int vrefInt;
        if(!nh.getParam("/vrefInt", vrefInt)) {
            ROS_ERROR("Failed to get param 'vrefInt'");
            vrefInt = 25;
        }
        srv.request.vrefName = std::to_string(vrefInt);
        if(go_to_multiple_client.waitForExistence(ros::Duration(5))) {
            ROS_INFO("go_to_multiple service found");
        } else {
            ROS_INFO("go_to_multiple service not found after 5 seconds");
            return false;
        }
        if(go_to_multiple_client.call(srv)) {
            std::vector<double> state_refs_v(srv.response.state_refs.data.begin(), srv.response.state_refs.data.end()); // N by 3
            std::vector<double> input_refs_v(srv.response.input_refs.data.begin(), srv.response.input_refs.data.end()); // N by 2
            std::vector<double> wp_attributes_v(srv.response.wp_attributes.data.begin(), srv.response.wp_attributes.data.end()); // N by 1
            std::vector<double> wp_normals_v(srv.response.wp_normals.data.begin(), srv.response.wp_normals.data.end()); // N by 2
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

    utils::Point2D tuple_to_point(const std::tuple<float, float>& p) {
        utils::Point2D pt;
        pt.x = std::get<0>(p);
        pt.y = std::get<1>(p);
        return pt;
    }
    
    void remove_large_yaw_jump() {
        for (int i = 2; i < state_refs.rows(); i++) {
            double diff = state_refs(i, 2) - state_refs(i-1, 2);
            if (std::abs(diff) > 1 && std::abs(diff) < 5) {
                state_refs(i, 2) = 2 * state_refs(i-1, 2) - state_refs(i-2, 2);
            }
        }
        for (int i = 1; i < state_refs.rows(); i++) {
            double diff = state_refs(i, 2) - state_refs(i-1, 2);
            while (diff > M_PI) {
                state_refs(i, 2) -= 2 * M_PI;
                diff = state_refs(i, 2) - state_refs(i-1, 2);
            }
            while (diff < -M_PI) {
                state_refs(i, 2) += 2 * M_PI;
                diff = state_refs(i, 2) - state_refs(i-1, 2);
            }
        }
    }
    static Eigen::MatrixXd smooth_yaw_angles(const Eigen::MatrixXd& state_refs) {
        int N = state_refs.rows();
        if (N == 0) return state_refs; // Return empty if no data
        
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
        
        // Compute the smoothed yaw angles
        Eigen::VectorXd smooth_yaw(N);
        smooth_yaw(0) = yaw_angles(0);
        for (int i = 1; i < N; ++i) {
            smooth_yaw(i) = smooth_yaw(i - 1) + diffs(i - 1);
        }
        
        // Update the yaw column in the result matrix
        smoothed_refs.col(2) = smooth_yaw;
        
        return smoothed_refs;
    }
    bool set_params(const std::shared_ptr<TcpClient>& tcp_client) {
        std::vector<double> state_refs_v(state_refs.data(), state_refs.data() + state_refs.size());
        nh.setParam("/state_refs", state_refs_v);
        std::vector<double> state_attributes_v(state_attributes.data(), state_attributes.data() + state_attributes.size());
        nh.setParam("/state_attributes", state_attributes_v);
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

        /* if (trigger_client.call(trigger_srv)) { */
        /*     if (trigger_srv.response.success) { */
        /*         ROS_INFO("Python node notified successfully."); */
        /*     } else { */
        /*         ROS_WARN("Python node notification failed: %s", trigger_srv.response.message.c_str()); */
        /*     } */
        /* } else { */
        /*     ROS_ERROR("Failed to call the notification service."); */
        /* } */
        return true;
    }
    template <typename EigenType>
    static void saveToFile(const EigenType &data, const std::string &filename) {
        std::string dir = helper::getSourceDirectory();
        std::string file_path = dir + "/" + filename;
        std::ofstream file(file_path);
        if (file.is_open()) {
            file << data << "\n";
        } else {
            std::cerr << "Unable to open file: " << filename << std::endl;
        }
        file.close();
        std::cout << "Saved to " << file_path << std::endl;
    }
    static Eigen::MatrixXd loadTxt(const std::string &filename) {
        std::ifstream file(filename);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open file: " + filename);
        }

        std::string line;
        std::vector<double> matrixEntries;
        int numRows = 0;
        int numCols = -1;

        while (std::getline(file, line)) {
            std::istringstream iss(line);
            double num;
            std::vector<double> lineEntries;

            while (iss >> num) {
                lineEntries.push_back(num);
            }

            if (numCols == -1) {
                numCols = lineEntries.size();
            } else if (lineEntries.size() != numCols) {
                throw std::runtime_error("Inconsistent number of columns");
            }

            matrixEntries.insert(matrixEntries.end(), lineEntries.begin(), lineEntries.end());
            numRows++;
        }

        // Use Eigen::Map with row-major layout
        return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), numRows, numCols);
    }

};
#endif // PathManager_HPP
