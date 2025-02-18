#include <ros/ros.h>
#include <thread>
#include <map>
#include <string>
#include <vector>
#include <mutex>
#include <chrono>
#include "std_srvs/SetBoolRequest.h"
#include "utility.hpp"
#include "PathManager.hpp"
#include "MPC.hpp"
#include <signal.h>
#include <fstream>
#include <iostream>
#include "utils/waypoints.h"
#include "utils/goto_command.h"
#include "utils/set_states.h"
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <ncurses.h>
#include <std_msgs/Byte.h>
#include "utils/helper.h"

using namespace VehicleConstants;

class StateMachine {
public:
    StateMachine(ros::NodeHandle& nh_, double T, int N, double v_ref, bool sign, bool ekf, bool lane, double T_park, std::string robot_name, double x_init, double y_init, double yaw_init, bool real, bool use_beta): 
    nh(nh_), utils(nh, real, x_init, y_init, yaw_init, sign, ekf, lane, robot_name), mpc(T,N,v_ref,use_beta), path_manager(nh,T,N,v_ref),
    state(STATE::INIT), sign(sign), ekf(ekf), lane(lane), T_park(T_park), T(T), real(real)
    {
        // tunables
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
        if (keyboardControl) {
            utils.debug("keyboard control enabled", 2);
            change_state(STATE::KEYBOARD_CONTROL);
        }
        success = success && nh.getParam("/dashboard", dashboard);
        success = success && nh.getParam("/gps", hasGps);
        if (!success) {
            ROS_ERROR("Failed to get parameters");
            ros::shutdown();
        }

        //initialize parking spots
        for(int i=0; i<5; i++) {
            Eigen::Vector2d spot_right = {PARKING_SPOT_RIGHT[0] + i*PARKING_SPOT_LENGTH, PARKING_SPOT_RIGHT[1]};
            Eigen::Vector2d spot_left = {PARKING_SPOT_LEFT[0] + i*PARKING_SPOT_LENGTH, PARKING_SPOT_LEFT[1]};
            PARKING_SPOTS.push_back(spot_right);
            PARKING_SPOTS.push_back(spot_left);
        }

        double rateVal =  1/mpc.T;
        rate = new ros::Rate(rateVal);
        std::cout << "rate: " << rateVal << std::endl;
        goto_command_server = nh.advertiseService("/goto_command", &StateMachine::goto_command_callback, this);
        set_states_server = nh.advertiseService("/set_states", &StateMachine::set_states_callback, this);
        start_trigger = nh.advertiseService("/start_bool", &StateMachine::start_bool_callback, this);
        utils.debug("start_bool server ready, mpc time step T = " + helper::d2str(T), 2);
        utils.debug("state machine initialized", 2);

    }
    ~StateMachine() {
        // utils.stop_car();
    }
    ros::NodeHandle& nh;

// private:
    //tunables
    double  change_lane_yaw = 0.15, cw_speed_ratio, hw_speed_ratio, sign_localization_threshold = 0.5, 
            lane_localization_orientation_threshold = 10, pixel_center_offset = -30.0, constant_distance_to_intersection_at_detection = 0.371,
            intersection_localization_threshold = 0.5, stop_duration = 3.0, parking_base_yaw_target = 0.166, parking_base_speed=-0.2, parking_base_thresh=0.1,
            change_lane_speed=0.2, change_lane_thresh=0.05, sign_localization_orientation_threshold = 15, intersection_localization_orientation_threshold = 15,
            NORMAL_SPEED = 0.175,
            FAST_SPEED = 0.4, change_lane_offset_scaler = 1.2;
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
    
    ros::Rate* rate;

    std::mutex lock;
    Utility utils;
    PathManager path_manager;
    ros::ServiceServer goto_command_server, set_states_server;
    MPC mpc;

    // intersection variables
    Eigen::Vector2d last_intersection_point = {1000.0, 1000.0};
    Eigen::Vector2d next_intersection_point = {1000.0, 1000.0};

    void receive_services() {
        while(true) {
            if (utils.tcp_client == nullptr) {
                std::this_thread::sleep_for(std::chrono::milliseconds(10000));
                continue;
            }
            if (utils.tcp_client->get_go_to_cmd_srv_msgs().size() > 0) {
                std::vector<std::tuple<float, float>> coords = utils.tcp_client->get_go_to_cmd_srv_msgs().front()->coords;
                utils::goto_command::Response res;
                goto_multiple_command_callback(coords, res);
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
            if(utils.tcp_client->get_start_srv_msgs().size() > 0) {
                std_srvs::SetBool::Request req;
                std_srvs::SetBool::Response res;
                req.data = utils.tcp_client->get_start_srv_msgs().front();
                start_bool_callback(req, res);
                utils.tcp_client->send_start_srv(true);
                utils.tcp_client->get_start_srv_msgs().pop();
            }
            if(utils.tcp_client->get_waypoints_srv_msgs().size() > 0) {
                double x0 = utils.tcp_client->get_waypoints_srv_msgs().front()->x0;
                double y0 = utils.tcp_client->get_waypoints_srv_msgs().front()->y0;
                double yaw0 = utils.tcp_client->get_waypoints_srv_msgs().front()->yaw0;
                path_manager.call_waypoint_service(x0, y0, yaw0, utils.tcp_client);
                utils.tcp_client->get_waypoints_srv_msgs().pop();
            }
            std::this_thread::sleep_for(std::chrono::milliseconds(100));
        }
    
    }
    void call_trigger_service() {
        ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/trigger_service");
        std_srvs::Trigger srv;
        client.call(srv);
    }
    int initialize() {
        if (initialized) return 1;
        // sleep 2 seconds to allow for initialization
        ros::Duration(2).sleep();
        if(hasGps) {
            if(!utils.reinitialize_states()) ROS_WARN("Failed to reinitialize");
        }
        double x, y, yaw;
        utils.get_states(x, y, yaw);
        utils.update_states(x_current);
        utils.debug("start(): x=" + helper::d2str(x) + ", y=" + helper::d2str(y) + ", yaw=" + helper::d2str(yaw), 2);
        path_manager.call_waypoint_service(x, y, yaw, utils.tcp_client);
        destination = path_manager.state_refs.row(path_manager.state_refs.rows()-1).head(2);
        utils.debug("initialize(): start: " + helper::d2str(x) + ", " + helper::d2str(y), 2);
        utils.debug("initialize(): destination: " + helper::d2str(destination(0)) + ", " + helper::d2str(destination(1)), 2);

        path_manager.target_waypoint_index = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1); // search from the beginning to the end
        mpc.reset_solver();
        initialized = true;
        return 1;
    }
    int start() {
        static int testing = 2;
        if (testing == 2) {
            nh.getParam("/test", testing);
        }
        if (testing) {
            change_state(STATE::TESTING);
        } else {
            // find_next_intersection(true);
            change_state(STATE::MOVING);
        }
        return 1;
    }
    bool start_bool_callback(std_srvs::SetBool::Request &req, std_srvs::SetBool::Response &res) {
        static int history = -1;
        if (req.data && (state == STATE::INIT || state == STATE::DONE)) {
            initialize();
            start();
            res.success = true;
            res.message = "Started";
        } else {
            history = state;
            stop_for(10*T);
            change_state(STATE::INIT);
            res.success = true;
            res.message = "Stopped";
            stop_for(10*T);
        }
        return true;
        
    }
    ros::ServiceServer start_trigger;
    void solve();
    void publish_commands();
    void update_mpc_states();
    void update_mpc_states(double x, double y, double yaw) {
        if(path_manager.closest_waypoint_index < path_manager.state_refs.rows() && path_manager.state_refs.rows() > 0) {
            double ref_yaw = path_manager.state_refs(path_manager.closest_waypoint_index, 2);
            yaw = Utility::yaw_mod(yaw, ref_yaw);
        }
        x_current << x, y, yaw;
    }
    void change_state(STATE new_state);
    void run();
    void stop_for(double duration) {
        ros::Time timer = ros::Time::now() + ros::Duration(duration);
        while (ros::Time::now() < timer) {
            utils.publish_cmd_vel(0.0, 0.0);
            rate->sleep();
        }
        // TODO: reset path manager idx
    }
    int parking_maneuver_hardcode(bool right=true, bool exit=false, double rate_val=20, double initial_y_error = 0, double initial_yaw_error = 0) {
        // rate_val = 1/mpc.T;
        Eigen::VectorXd targets(3);
        Eigen::VectorXd steerings(3);
        Eigen::VectorXd speeds(3);
        Eigen::VectorXd thresholds(3);
        double base_yaw_target = parking_base_yaw_target * M_PI;
        utils.debug("parking_maneuver_hardcode(): base yaw target: " + helper::d2str(base_yaw_target / M_PI) + "pi", 3);
        base_yaw_target = base_yaw_target + 0.02 / (0.29 * M_PI) * base_yaw_target * initial_y_error / MAX_PARKING_Y_ERROR * (right ? 1 : -1);
        utils.debug("parking_maneuver_hardcode(): initial y error: " + helper::d2str(initial_y_error) + ", initial yaw error: " + helper::d2str(initial_yaw_error) + ", base yaw target: " + helper::d2str(base_yaw_target / M_PI) + "pi", 2);
        double base_steer = - HARD_MAX_STEERING;
        double base_speed = parking_base_speed;
        double base_thresh = parking_base_thresh;
        targets << base_yaw_target, 0.0, 0.0;
        steerings << -base_steer, base_steer, -base_steer;
        speeds << base_speed, base_speed, -base_speed;
        thresholds << base_thresh, base_thresh, base_thresh/3;
        if (exit) {
            base_yaw_target *= 0.95;
            targets << base_yaw_target, base_yaw_target, 0.0;
            thresholds(0) = (1 - base_thresh / base_yaw_target) * base_yaw_target;
            thresholds(1) = base_thresh;
            thresholds(2) = base_thresh;
            speeds << base_speed, -base_speed, -base_speed;
        }
        if (!right) {
            steerings *= -1;
            targets *= -1;
            std::cout << "targets: " << targets.transpose() << ", steerings: " << steerings.transpose() << ", speeds: " << speeds.transpose() << ", thresholds: " << thresholds.transpose() << std::endl;
        }
        utils.debug(std::string("parking_maneuver_hardcode(): park right: ") + 
            (right ? "true" : "false") + 
            ", exit: " + helper::d2str(exit), 2);
        std::cout << "targets: " << targets.transpose() << ", steerings: " << steerings.transpose() << ", speeds: " << speeds.transpose() << ", thresholds: " << thresholds.transpose() << std::endl;
        return maneuver_hardcode(targets, steerings, speeds, thresholds, rate_val);
    }
    int change_lane_hardcode(bool right, double yaw_offset = 0, double rate_val = 20) {
        Eigen::VectorXd targets(1);
        double yaw_target = change_lane_yaw * M_PI;
        Eigen::VectorXd steering(1);
        steering << -23.;
        if (right) {
            steering *= -1;
            yaw_target *= -1;
        }
        yaw_target += yaw_offset;
        targets << yaw_target;
        Eigen::VectorXd speeds(1);
        speeds << change_lane_speed;
        Eigen::VectorXd thresholds(1);
        thresholds << change_lane_thresh;
        return maneuver_hardcode(targets, steering, speeds, thresholds, rate_val);
    }
    int maneuver_hardcode(const Eigen::VectorXd &targets, const Eigen::VectorXd &steerings, const Eigen::VectorXd &speeds, const Eigen::VectorXd &thresholds, double rate_val = 20) {
        /* 
        targets: target yaws or distances
        thresholds: thresholds for each target        
        */
        double x0, y0, yaw0;
        // get current states
        utils.get_states(x0, y0, yaw0);
        // get closest direction
        yaw0 = Utility::nearest_direction(yaw0);
        yaw0 = Utility::yaw_mod(yaw0);
        Eigen::VectorXd yaw0_vec = Eigen::VectorXd::Constant(targets.size(), yaw0);
        Eigen::VectorXd target_yaws = targets + yaw0_vec;
        // std::cout << "target yaws: " << target_yaws.transpose() << std::endl;
        utils.debug("maneuver_hardcode(): initial yaw: " + helper::d2str(yaw0) + ", target yaws: " + helper::d2str(target_yaws(0)) + ", " + helper::d2str(target_yaws(1)), 2);
        utils.debug("maneuver_hardcode(): nearest direction: " + helper::d2str(yaw0), 2);
        int stage = 1;
        int num_stages = targets.size();
        ros::Rate temp_rate(rate_val);
        
        double steering_angle = steerings(stage-1);
        double speed = speeds(stage-1);
        double yaw = utils.get_yaw();
        double yaw_error = yaw - target_yaws(stage-1);
        double yaw_error_sign = yaw_error > 0 ? 1 : -1;
        while(1) {
            yaw = utils.get_yaw();
            yaw = Utility::yaw_mod(yaw);
            yaw_error = yaw - target_yaws(stage-1);
            utils.debug("maneuver_hardcode(): stage " + helper::d2str(stage) + ", yaw: " + helper::d2str(yaw) + ", target yaw: " + helper::d2str(target_yaws(stage-1)) + ", yaw error: " + helper::d2str(yaw_error), 5);
            while(std::abs(yaw_error) > M_PI * 1.2) {
                if(yaw_error > M_PI * 1.2) {
                    yaw_error -= 2*M_PI;
                } else {
                    yaw_error += 2*M_PI;
                }
            }
            bool exit_cond;
            if (std::abs(steering_angle) < 0.1) {
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double dist_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                exit_cond = std::abs(dist_sq - targets(stage-1) * targets(stage-1)) < thresholds(stage-1) * thresholds(stage-1);
                orientation_follow(yaw0, speed);
            } else {
                if(yaw_error_sign < 0) {
                    exit_cond = yaw_error > -thresholds(stage-1);
                } else {
                    exit_cond = yaw_error < thresholds(stage-1);
                }
                utils.debug("maneuver_hardcode(): yaw error: " + helper::d2str(yaw_error) + ", exit condition: " + helper::d2str(exit_cond), 5);
                utils.publish_cmd_vel(steering_angle, speed);
            }
            if (exit_cond) {
                utils.debug("stage " + helper::d2str(stage) + " completed. yaw error: " + helper::d2str(yaw_error), 3);
                stage++;
                if (stage > num_stages) {
                    utils.debug("maneuver completed", 2);
                    break;
                }
                steering_angle = steerings(stage-1);
                speed = speeds(stage-1);
                utils.debug("new stage: " + helper::d2str(stage) + ", steer: " + helper::d2str(steering_angle) + ", speed: " + helper::d2str(speed), 3);
                yaw_error = yaw - target_yaws(stage-1);
                yaw_error_sign = yaw_error > 0 ? 1 : -1;
                continue;
            }
            temp_rate.sleep();
        }
        return 0;
    }

    bool near_intersection() {
        double yaw = utils.get_yaw();
        double nearest_direction = Utility::nearest_direction(yaw);
        double yaw_error = nearest_direction - yaw;
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        if(std::abs(yaw_error) > 45 * M_PI / 180) {
            utils.debug("near_intersection(): FAILURE: yaw error too large: " + helper::d2str(yaw_error), 4);
            return false;
        }

        int nearestDirectionIndex = Utility::nearest_direction_index(yaw);
        const auto& direction_intersections = (nearestDirectionIndex == 0) ? EAST_FACING_INTERSECTIONS :
                                          (nearestDirectionIndex == 1) ? NORTH_FACING_INTERSECTIONS :
                                          (nearestDirectionIndex == 2) ? WEST_FACING_INTERSECTIONS :
                                                                        SOUTH_FACING_INTERSECTIONS;
        
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
            utils.debug("near_intersection(): SUCCESS: estimated intersection position: (" + helper::d2str(estimated_position[0]) + ", " + helper::d2str(estimated_position[1]) + "), actual: (" + helper::d2str(direction_intersections[min_index][0]) + ", " + helper::d2str(direction_intersections[min_index][1]) + "), error: (" + helper::d2str(direction_intersections[min_index][0] - estimated_position[0]) + ", " + helper::d2str(direction_intersections[min_index][1] - estimated_position[1]) + ")", 4);
            return true;
        } else {
            utils.debug("near_intersection(): FAILURE: estimated intersection position: (" + helper::d2str(estimated_position[0]) + ", " + helper::d2str(estimated_position[1]) + "), actual: (" + helper::d2str(direction_intersections[min_index][0]) + ", " + helper::d2str(direction_intersections[min_index][1]) + "), error: (" + helper::d2str(direction_intersections[min_index][0] - estimated_position[0]) + ", " + helper::d2str(direction_intersections[min_index][1] - estimated_position[1]) + ")", 4);
            return false;
        }
    }
    bool find_next_intersection(bool first = false) {
        int closest_idx = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1);
        double threshold = INTERSECTION_DISTANCE_THRESHOLD;
        if (first) {
            threshold = 0.0;
        }
        int next_intersection_idx = closest_idx + static_cast<int>(threshold * path_manager.density);
        int limit = std::max(next_intersection_idx + static_cast<int>(30.0 * path_manager.density), static_cast<int>(path_manager.state_refs.rows() - 1));
        while(true) {
            if (path_manager.attribute_cmp(next_intersection_idx, path_manager.ATTRIBUTE::STOPLINE)) {
                break;
            }
            if (next_intersection_idx >= limit) {
                utils.debug("find_next_intersection(): FAILURE: no waypoint of attribute stopline found", 1);
                return false;
            }
            next_intersection_idx++;
        }
        double closest_yaw = path_manager.state_refs(next_intersection_idx, 2);
        double nearest_direction = Utility::nearest_direction(closest_yaw);
        auto& INTERSECTIONS = (nearest_direction == 0) ? EAST_FACING_INTERSECTIONS :
                                (nearest_direction == 1) ? NORTH_FACING_INTERSECTIONS :
                                (nearest_direction == 2) ? WEST_FACING_INTERSECTIONS :
                                                            SOUTH_FACING_INTERSECTIONS;
        bool found = false;
        double x = path_manager.state_refs(next_intersection_idx, 0);
        double y = path_manager.state_refs(next_intersection_idx, 1);
        for(auto& intersection : INTERSECTIONS) {
            double dist_sq = std::pow(intersection[0] - x, 2) + std::pow(intersection[1] - y, 2);
            if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD * INTERSECTION_DISTANCE_THRESHOLD) {
                utils.debug("find_next_intersection(): found intersection at (" + helper::d2str(intersection[0]) + ", " + helper::d2str(intersection[1]) + ")", 1);
                next_intersection_point = {intersection[0], intersection[1]};
                found = true;
                break;
            }
        }
        if (!found) {
            utils.debug("find_next_intersection(): FAILURE: no intersection near (" + helper::d2str(x) + ", " + helper::d2str(y) + ")", 1);
            return false;
        }
        return found;
    }
    bool intersection_reached() {
        static double lookahead_dist = 0.15;
        static int num_index = static_cast<int>(lookahead_dist * path_manager.density);
        if(lane && use_stopline) {
            if (utils.stopline > 0)
            {
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
                //         utils.debug("intersection_reached(): closest idx + i = " + helper::d2str(closest_idx + i) + " exceeds path_manager.state_refs.rows(): " + helper::d2str(path_manager.state_refs.rows()), 2);
                //         break;
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
                        utils.debug("intersection_reached(): intersection detected, but distance (" + helper::d2str(std::sqrt(dist_sq)) + ") too close to previous intersection, ignoring...", 4);
                        return false;
                    }
                    utils.debug("intersection_reached(): setting last intersection point to (" + helper::d2str(x) + ", " + helper::d2str(y) + ")", 2);
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
            } else return false;
        }
        utils.update_states(x_current);
        // utils.get_states(running_x, running_y, running_yaw);
        update_mpc_states(x_current[0], x_current[1], x_current[2]);
        int target_index = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1);
        bool found = false;
        for (int i = 0; i < num_index; i++) {
            if (target_index + i >= path_manager.state_refs.rows()) break;
            if(path_manager.attribute_cmp(target_index+i, path_manager.ATTRIBUTE::STOPLINE)) {
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
    bool sign_in_path(int sign_idx, double search_dist) {
        auto estimated_sign_pose = utils.object_world_pose(sign_idx);
        double x = estimated_sign_pose[0];
        double y = estimated_sign_pose[1];
        int closest_idx = path_manager.closest_waypoint_index;
        int num_index = static_cast<int>(search_dist * path_manager.density);
        double min_dist_sq = std::numeric_limits<double>::max();
        double threshold = INTERSECTION_TO_SIGN * INTERSECTION_TO_SIGN * 1.5 * 1.5;
        for (int i = closest_idx; i < closest_idx + num_index; i+=4) {
            if (i >= path_manager.state_refs.rows()) break;
            double dist_sq = std::pow(x - path_manager.state_refs(i, 0), 2) + std::pow(y - path_manager.state_refs(i, 1), 2);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
            }
            if (min_dist_sq < threshold) {
                utils.debug("sign_in_path(): sign at (" + helper::d2str(x) + ", " + helper::d2str(y) + ") found in path", 1);
                return true;
            }
        }
        return false;
    }
    void check_light() {
        static bool wait_for_green_flag = false;
        if (stopsign_flag == OBJECT::NONE) wait_for_green_flag = false;
        utils.update_states(x_current);
        double &x = x_current[0];
        double &y = x_current[1];
        double dist_sq = std::pow(x - last_intersection_point(0), 2) + std::pow(y - last_intersection_point(1), 2);
        if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD/1.5 * INTERSECTION_DISTANCE_THRESHOLD/1.5) {
            // distance to last intersection too close
            return;
        }
        // check for traffic light
        static bool relocalized = false;
        // if (stopsign_flag != OBJECT::NONE && relocalized) return; // sign already detected
        if (stopsign_flag == OBJECT::NONE) relocalized = false;
        static Eigen::Vector2d light_pose(1000.0, 1000.0);
        if (stopsign_flag != OBJECT::NONE && stopsign_flag != OBJECT::LIGHTS && stopsign_flag != OBJECT::REDLIGHT 
            && stopsign_flag != OBJECT::GREENLIGHT && stopsign_flag != OBJECT::YELLOWLIGHT)
        { 
            return; // sign already detected 
        }
        double dist = -10.0;
        bool is_red = false;
        int sign_index = utils.object_index(OBJECT::REDLIGHT);
        if(sign_index >= 0) {
            is_red = true;
        } else {
            sign_index = utils.object_index(OBJECT::LIGHTS);
        }
        if(sign_index >= 0) is_red = true; // consider LIGHTS as red light
        if (sign_index < 0) sign_index = utils.object_index(OBJECT::GREENLIGHT);
        if (sign_index < 0) sign_index = utils.object_index(OBJECT::YELLOWLIGHT);
        if(sign_index >= 0 && !relocalized && stopsign_flag == OBJECT::NONE) {
            dist = utils.object_distance(sign_index);
            if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
                detected_dist = dist;
                light_pose = utils.object_world_pose(sign_index);
                // double dist_to_last_intersection_sq = std::pow(light_pose[0] - last_intersection_point[0], 2) + std::pow(light_pose[1] - last_intersection_point[1], 2);
                // if (dist_to_last_intersection_sq < std::pow(INTERSECTION_TO_SIGN * 2, 2)) {
                //     utils.debug("check_light(): traffic light detected too close to last intersection, ignoring...", 2);
                //     return;
                // }
                if (sign_in_path(sign_index, dist + 0.2)) {
                    utils.debug("check_light(): traffic light detected at a distance of: " + helper::d2str(dist), 2);
                    stopsign_flag = OBJECT::LIGHTS;
                    if (sign_relocalize) {
                        std::string sign_type;
                        const auto& intersection_signs = utils.get_relevant_signs(stopsign_flag, sign_type);
                        relocalized = sign_based_relocalization(light_pose, intersection_signs, sign_type);
                    }
                }
            }
        }   
        if (is_red && dist < MAX_LIGHT_DIST && !wait_for_green_flag) {
            mpc.reset_solver();
            wait_for_green();
            wait_for_green_flag = true;
        }
    }
    void check_stop_sign() {
        static bool relocalized = false;
        if (stopsign_flag != OBJECT::NONE && relocalized) return; // sign already detected 
        if (stopsign_flag == OBJECT::NONE) relocalized = false;
        utils.update_states(x_current);
        double &x = x_current[0];
        double &y = x_current[1];
        double dist_sq = std::pow(x - last_intersection_point(0), 2) + std::pow(y - last_intersection_point(1), 2);
        if (dist_sq < INTERSECTION_DISTANCE_THRESHOLD/1.5 * INTERSECTION_DISTANCE_THRESHOLD/1.5) {
            // distance to last intersection too close
            return;
        }
        
        int sign_index;
        double dist = -10.0;

        // check for stop sign
        sign_index = utils.object_index(OBJECT::STOPSIGN);
        if (stopsign_flag == OBJECT::NONE) { // if no sign detected
            if(sign_index >= 0) {
                dist = utils.object_distance(sign_index);
                
                if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
                    detected_dist = dist;
                    if (sign_in_path(sign_index, dist + 0.2)) {
                        utils.debug("check_stop_sign(): stop sign detected at a distance of: " + helper::d2str(dist), 2);
                        stopsign_flag = OBJECT::STOPSIGN;
                    }
                }
            }
        }
        
        // check for priority sign
        if (stopsign_flag == OBJECT::NONE) {
            sign_index = utils.object_index(OBJECT::PRIORITY);
            if(sign_index >= 0) {
                dist = utils.object_distance(sign_index);
                if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
                    detected_dist = dist;
                    if (sign_in_path(sign_index, dist + 0.2)) {
                        utils.debug("check_stop_sign(): priority detected at a distance of: " + helper::d2str(dist), 2);
                        stopsign_flag = OBJECT::PRIORITY;
                    }
                }
            }
        }

        // check for roundabout sign
        if (stopsign_flag == OBJECT::NONE) {
            sign_index = utils.object_index(OBJECT::ROUNDABOUT);
            if(sign_index >= 0) {
                dist = utils.object_distance(sign_index);
                if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
                    detected_dist = dist;
                    if (sign_in_path(sign_index, dist + 0.2)) {
                        utils.debug("check_stop_sign(): roundabout detected at a distance of: " + helper::d2str(dist), 2);
                        stopsign_flag = OBJECT::ROUNDABOUT;
                    }
                }
            }
        }
        
        // check for crosswalk
        if (stopsign_flag == OBJECT::NONE) {
            sign_index = utils.object_index(OBJECT::CROSSWALK);
            if(sign_index >= 0) {
                dist = utils.object_distance(sign_index);
                if (dist < MAX_SIGN_DIST && dist > MIN_SIGN_DIST) {
                    detected_dist = dist;
                    if (sign_in_path(sign_index, dist + 0.2)) {
                        utils.debug("check_stop_sign(): crosswalk detected at a distance of: " + helper::d2str(dist), 2);
                        stopsign_flag = OBJECT::CROSSWALK;
                    }
                }
            }
        }
        
        // relocalize based on sign
        if (sign_relocalize && stopsign_flag != OBJECT::NONE && stopsign_flag != OBJECT::LIGHTS && stopsign_flag != OBJECT::REDLIGHT && stopsign_flag != OBJECT::GREENLIGHT && stopsign_flag != OBJECT::YELLOWLIGHT) {
            auto sign_pose = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], utils.object_box(sign_index), detected_dist);
            std::string sign_type;
            const auto& intersection_signs = utils.get_relevant_signs(stopsign_flag, sign_type);
            relocalized = sign_based_relocalization(sign_pose, intersection_signs, sign_type);
        }
    }
    int park_sign_detected() {
        int park_index = utils.object_index(OBJECT::PARK);
        if(park_index >= 0) {
            double dist = utils.object_distance(park_index);
            if (dist < MAX_PARK_DIST && dist > 0) {
                detected_dist = dist;
                return park_index;
            }
        }
        return -1;
    }
    double check_crosswalk() {
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
                utils.debug("crosswalk detected at a distance of: " + helper::d2str(detected_dist), 2);
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
    bool check_highway_signs() {
        if (!sign_relocalize) return false;
        static ros::Time hw_cooldown_timer = ros::Time::now();
        static double detected_dist = -1;
        static int detection_count = 0;
        if(hw_cooldown_timer > ros::Time::now()) {
            return false;
        }
        bool is_exit = false;
        int hw_index = utils.object_index(OBJECT::HIGHWAYENTRANCE);
        if (hw_index < 0) {
            hw_index = utils.object_index(OBJECT::HIGHWAYEXIT);
            is_exit = true;
        }
        if(hw_index >= 0) {
            detected_dist = utils.object_distance(hw_index);
            // relocalize once at MAX_SIGN_DIST2, then again at MAX_SIGN_DIST
            double distance_threshold = detection_count == 0 ? MAX_SIGN_DIST2 : MAX_SIGN_DIST;
            if (detected_dist < distance_threshold && detected_dist > MIN_SIGN_DIST) {
                std::string hw_type = is_exit ? "exit" : "entrance";
                utils.update_states(x_current);
                auto hw_pose = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], utils.object_box(hw_index), detected_dist);
                std::string sign_type;
                const auto& direction_highways = utils.get_relevant_signs(is_exit ? OBJECT::HIGHWAYEXIT : OBJECT::HIGHWAYENTRANCE, sign_type);
                bool success = sign_based_relocalization(hw_pose, direction_highways, sign_type);
                // utils.
                detection_count++;
                if (detection_count >= 2) {
                    double cd = (detected_dist + 4) / FAST_SPEED;
                    hw_cooldown_timer = ros::Time::now() + ros::Duration(cd);
                    detection_count = 0;
                }
                return success;
            }
        }
        return false;
    }
    double check_highway() {
        utils.update_states(x_current);
        update_mpc_states(x_current[0], x_current[1], x_current[2]);
        int closest_idx = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1);
        return path_manager.attribute_cmp(closest_idx, path_manager.ATTRIBUTE::HIGHWAYLEFT) || path_manager.attribute_cmp(closest_idx, path_manager.ATTRIBUTE::HIGHWAYRIGHT);
    }
    void check_emergency_stop() {
        if (!emergency) return;
        if (utils.emergency) {
            utils.debug("check_emergency_stop(): emergency stop triggered", 1);
            while (utils.emergency) {
                stop_for(T);
                rate->sleep();
            }
            stop_for(stop_duration);
            while (utils.emergency) { // double check
                stop_for(T);
                rate->sleep();
            }
            utils.debug("check_emergency_stop(): emergency stop released", 1);
        }
    }
    void pedestrian_detected() {
        int pedestrian_count = 0;
        bool detected;
        if (real) {
            detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
        } else {
            // detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0 || utils.object_index(OBJECT::HIGHWAYEXIT) >= 0;
            detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
        }
        if (detected) {
            mpc.reset_solver();
            while (true) {
                if (real) {
                    detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
                } else {
                    detected = utils.object_index(OBJECT::PEDESTRIAN) >= 0;
                }
                if (detected) {
                    double dist;
                    if (real) dist = utils.object_distance(utils.object_index(OBJECT::HIGHWAYEXIT));
                    else dist = utils.object_distance(utils.object_index(OBJECT::PEDESTRIAN));
                    utils.debug("pedestrian_detected(): girl detected at a distance of: " + helper::d2str(dist), 2);
                    stop_for(stop_duration);
                } else {
                    pedestrian_count ++;
                    utils.debug("pedestrian_detected(): pedestrian sem: " + helper::d2str(pedestrian_count)+ " out of " + helper::d2str(pedestrian_count_thresh), 2);
                    stop_for(stop_duration/15);
                }
                if (pedestrian_count > pedestrian_count_thresh) break;
            }
            rate->sleep();
        }
    }
    void exit_detected() {
        // if (utils.object_index(OBJECT::NOENTRY) >= 0) {
        //     ROS_INFO("no entry detected, signaling end of mission");
        //     change_state(STATE::DONE);
        // }
    }
    int sign_based_relocalization(const Eigen::Vector2d estimated_sign_pose, const std::vector<std::vector<double>> &EMPIRICAL_POSES, const std::string& sign_type = "") {
        int min_index = 0;
        double min_error_sq = 1000.0;
        if (utils.get_min_object_index(estimated_sign_pose, EMPIRICAL_POSES, min_index, min_error_sq, sign_localization_threshold)) {
            double yaw = utils.get_yaw();
            double sign_direction = EMPIRICAL_POSES[min_index][2];
            double yaw_error = Utility::compare_yaw(sign_direction, yaw);
            if(yaw_error > sign_localization_orientation_threshold * M_PI / 180) {
                utils.debug("SIGN_RELOC(" + sign_type + "): FAILURE: yaw error too large: " + helper::d2str(yaw_error) + ", threshold: " + helper::d2str(sign_localization_orientation_threshold), 2);
                return 0;
            }
            utils.debug("SIGN_RELOC(" + sign_type + "): SUCCESS: estimated sign pose: (" + helper::d2str(estimated_sign_pose[0]) + ", " + helper::d2str(estimated_sign_pose[1]) + "), actual: (" + helper::d2str(EMPIRICAL_POSES[min_index][0]) + ", " + helper::d2str(EMPIRICAL_POSES[min_index][1]) + "), error: (" + helper::d2str(EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0]) + ", " + helper::d2str(EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]) + "), error norm: " + helper::d2str(std::sqrt(min_error_sq)) + ", threshold: " + helper::d2str(sign_localization_threshold), 2);
            double x,y;
            utils.get_states(x, y, yaw);
            utils.recalibrate_states(EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0], EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]);
            utils.update_states(x_current);
            utils.debug("SIGN_RELOC(" + sign_type + "): old states: (" + helper::d2str(x) + ", " + helper::d2str(y) + "), new states: (" + helper::d2str(x_current[0]) + ", " + helper::d2str(x_current[1]) + ")", 2);
            path_manager.reset_target_waypoint_index(x_current);
            mpc.reset_solver();
            return 1;
        } else {
            utils.debug("SIGN_RELOC(" + sign_type + "): FAILURE: error too large: " + helper::d2str(std::sqrt(min_error_sq)) + ", threshold: " + helper::d2str(sign_localization_threshold) + ", estimated sign pose: (" + helper::d2str(estimated_sign_pose[0]) + ", " + helper::d2str(estimated_sign_pose[1]) + ")", 2);
            return 0;
        }
    }
    int intersection_based_relocalization() {
        // stop_for(0.5);
        // check orientation
        double yaw = utils.get_yaw();
        double nearest_direction = Utility::nearest_direction(yaw);
        double yaw_error = nearest_direction - yaw;
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        if(std::abs(yaw_error) > intersection_localization_orientation_threshold * M_PI / 180) {
            utils.debug("intersection_based_relocalization(): FAILURE: yaw error too large: " + helper::d2str(yaw_error), 2);
            return 0;
        }

        int nearestDirectionIndex = Utility::nearest_direction_index(yaw);
        const auto& direction_intersections = (nearestDirectionIndex == 0) ? EAST_FACING_INTERSECTIONS :
                                          (nearestDirectionIndex == 1) ? NORTH_FACING_INTERSECTIONS :
                                          (nearestDirectionIndex == 2) ? WEST_FACING_INTERSECTIONS :
                                                                        SOUTH_FACING_INTERSECTIONS;
        
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
            utils.debug("intersection based relocalization(): SUCCESS: estimated intersection position: (" + helper::d2str(estimated_position[0]) + ", " + helper::d2str(estimated_position[1]) + "), actual: (" + helper::d2str(direction_intersections[min_index][0]) + ", " + helper::d2str(direction_intersections[min_index][1]) + "), error: (" + helper::d2str(direction_intersections[min_index][0] - estimated_position[0]) + ", " + helper::d2str(direction_intersections[min_index][1] - estimated_position[1]) + ")", 2);
            utils.recalibrate_states(direction_intersections[min_index][0] - estimated_position[0], direction_intersections[min_index][1] - estimated_position[1]);
            return 1; // Successful relocalization
        } else {
            utils.debug("intersection based relocalization(): FAILURE: estimated intersection position: (" + helper::d2str(estimated_position[0]) + ", " + helper::d2str(estimated_position[1]) + "), actual: (" + helper::d2str(direction_intersections[min_index][0]) + ", " + helper::d2str(direction_intersections[min_index][1]) + "), error: (" + helper::d2str(direction_intersections[min_index][0] - estimated_position[0]) + ", " + helper::d2str(direction_intersections[min_index][1] - estimated_position[1]) + ")", 2);
            return 0; // Failed to relocalize
        }
    }

    int lane_based_relocalization() {
        static int count = 0;
        if (count >= 100) count = 0;
        count++;
        static ros::Time lane_cooldown_timer = ros::Time::now();
        static double cooldown = 3.0; // seconds
        if(lane_cooldown_timer > ros::Time::now()) {
            // utils.debug("LANE_RELOC(): FAILURE: on cooldown" + helper::d2str(count), 4);
            return 0;
        }
        static double lookahead = 0.5;
        static double lookbehind = 0.3;
        
        int closest_idx = path_manager.closest_waypoint_index;
        int end_idx = std::min(static_cast<int>(closest_idx + lookahead * path_manager.density), static_cast<int>(path_manager.state_refs.rows() - 1));
        int start_idx = std::max(0, closest_idx - static_cast<int>(lookbehind * path_manager.density));
        if (!path_manager.lane_detectable(closest_idx, end_idx)) {
            // utils.debug("LANE_RELOC(): FAILURE: lane not detectable" + helper::d2str(count), 4);
            return 0;
        }
        int num_waypoints = static_cast<int>((lookahead + lookbehind) * path_manager.density);
        double nearest_direction = Utility::nearest_direction(utils.get_yaw());
        if (!path_manager.is_straight_line(start_idx, num_waypoints, nearest_direction, 0.1)) {
            // utils.debug("LANE_RELOC(): FAILURE: not a straight line"+ helper::d2str(count), 4);
            return 0;
        }
        utils.update_states(x_current);
        double center = utils.center + pixel_center_offset;
        if (center < 180 || center > 460) {
            // utils.debug("LANE_RELOC(): FAILURE: center out of bounds: " + helper::d2str(center), 4);
            return 0;
        }
        double yaw = utils.get_yaw();
        double yaw_error = nearest_direction - yaw;
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        if (std::abs(yaw_error) > lane_localization_orientation_threshold * M_PI / 180) {
            utils.debug("LANE_RELOC(): FAILURE: yaw error too large: " + helper::d2str(yaw_error) + ", threshold: " + helper::d2str(lane_localization_orientation_threshold), 2);
            return 0;
        }
        double offset = (IMAGE_WIDTH/2 - center) / 80 * LANE_CENTER_TO_EDGE;
        int nearestDirectionIndex = Utility::nearest_direction_index(yaw);
        const auto& LANE_CENTERS = (nearestDirectionIndex == 0) ? EAST_FACING_LANE_CENTERS :
                                    (nearestDirectionIndex == 1) ? NORTH_FACING_LANE_CENTERS :
                                    (nearestDirectionIndex == 2) ? WEST_FACING_LANE_CENTERS :
                                                                SOUTH_FACING_LANE_CENTERS;
        if (nearestDirectionIndex == 0 || nearestDirectionIndex == 2) { // East, 0 || West, 2
            if (nearestDirectionIndex == 2) offset *= -1;
            int min_index = 0;
            double min_error = 1000;
            for (int i = 0; i < LANE_CENTERS.size(); i++) {
                double error = (LANE_CENTERS[i] - offset) - x_current[1];
                if (std::abs(error) < std::abs(min_error)) {
                    min_error = error;
                    min_index = i;
                }
            }
            if (std::abs(min_error) < LANE_OFFSET/2) {
                utils.recalibrate_states(0, min_error);
                utils.debug("LANE_RELOC(): SUCCESS: error: " + helper::d2str(min_error) + ", lane center: " + helper::d2str(LANE_CENTERS[min_index]) + ", offset: " + helper::d2str(offset) + ", nearest direction: " + helper::d2str(nearestDirectionIndex) + ", running y: " + helper::d2str(x_current[1]) + ", estimated running y: " + helper::d2str(LANE_CENTERS[min_index] - offset), 2);
                lane_cooldown_timer = ros::Time::now() + ros::Duration(cooldown);
                return 1;
            } else {
                utils.debug("LANE_RELOC(): FAILURE: error too large: " + helper::d2str(min_error) + ", lane center: " + helper::d2str(LANE_CENTERS[min_index]) + ", offset: " + helper::d2str(offset) + ", nearest direction: " + helper::d2str(nearestDirectionIndex) + ", running y: " + helper::d2str(x_current[1]) + ", estimated running y: " + helper::d2str(LANE_CENTERS[min_index] - offset), 2);
                return 0;
            }
        } else if (nearestDirectionIndex == 1 || nearestDirectionIndex == 3) { // North, 1 || South, 3
            if (nearestDirectionIndex == 3) offset *= -1;
            int min_index = 0;
            double min_error = 1000;
            for (int i = 0; i < LANE_CENTERS.size(); i++) {
                double error = (LANE_CENTERS[i] + offset) - x_current[0];
                if (std::abs(error) < std::abs(min_error)) {
                    min_error = error;
                    min_index = i;
                }
            }
            if (std::abs(min_error) < LANE_OFFSET/2) {
                utils.recalibrate_states(min_error, 0);
                lane_cooldown_timer = ros::Time::now() + ros::Duration(cooldown);
                utils.debug("LANE_RELOC(): SUCCESS: error: " + helper::d2str(min_error) + ", lane center: " + helper::d2str(LANE_CENTERS[min_index]) + ", offset: " + helper::d2str(offset) + ", nearest direction: " + helper::d2str(nearestDirectionIndex) + ", running x: " + helper::d2str(x_current[0]) + ", estimated running x: " + helper::d2str(LANE_CENTERS[min_index] + offset), 2);
                return 1;
            } else {
                utils.debug("LANE_RELOC(): FAILURE: error too large: " + helper::d2str(min_error) + ", lane center: " + helper::d2str(LANE_CENTERS[min_index]) + ", offset: " + helper::d2str(offset) + ", nearest direction: " + helper::d2str(nearestDirectionIndex) + ", running x: " + helper::d2str(x_current[0]) + ", estimated running x: " + helper::d2str(LANE_CENTERS[min_index] + offset), 2);
                return 0;
            }
        }
        return 0;
    }
    void wait_for_green() {
        if (has_light) {
            int neareastDirection = Utility::nearest_direction_index(utils.get_yaw());
            static std::string light_topic_name;
            if (neareastDirection == 0) light_topic_name = "/east_traffic_light";
            else if (neareastDirection == 1) light_topic_name = "/north_traffic_light";
            else if (neareastDirection == 2) light_topic_name = "/west_traffic_light";
            else if (neareastDirection == 3) light_topic_name = "/south_traffic_light";
            auto is_green = ros::topic::waitForMessage<std_msgs::Byte>(light_topic_name, ros::Duration(3));
            int n = 0;
            double ekf_x, ekf_y;
            double total_x, total_y;
            while (is_green->data != 1) {
                is_green = ros::topic::waitForMessage<std_msgs::Byte>(light_topic_name, ros::Duration(3));
                utils.publish_cmd_vel(0, 0);
                if (utils.useEkf) {
                    utils.get_ekf_states(ekf_x, ekf_y);
                    total_x += ekf_x;
                    total_y += ekf_y;
                    n++;
                }
                rate->sleep();
            }
            utils.debug("wait_for_green(): light turned green, proceeding...", 2);
            if (utils.useEkf) {
                utils.x0 = total_x / n - utils.odomX;
                utils.y0 = total_y / n - utils.odomY;
            }
            return;
        } else if (true) {
            utils.debug("wait_for_green(): red light detected, waiting for " + helper::d2str(stop_duration * 2) + "s or until light turns green", 2);
            auto expiring_time = ros::Time::now() + ros::Duration(3.0);
            int green_count = 0;
            while (true) {
                if (ros::Time::now() > expiring_time) {
                    utils.debug("wait_for_green(): timer expired, proceeding...", 2);
                    return;
                }
                int sign_index = utils.object_index(OBJECT::GREENLIGHT);
                if (sign_index >= 0) {
                    utils.debug("wait_for_green(): green light detected, proceeding...", 2);
                    green_count++;
                    if (green_count > 5) return;
                }
                if (sign_index < 0) sign_index = utils.object_index(OBJECT::YELLOWLIGHT);
                if (sign_index >= 0) {
                    utils.debug("wait_for_green(): yellow light detected, proceeding...", 2);
                    green_count++;
                    if (green_count > 5) return;
                }
                stop_for(T);
            }
            return;
        } else {
            utils.debug("wait_for_green(): light detected, but in simulation, proceeding ", 2);
            // stop_for(stop_duration);
        }
    }
    void publish_waypoints() {
        static Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(mpc.N, 3);
        path_manager.get_current_waypoints(waypoints);
        static std_msgs::Float32MultiArray msg;
        msg.data.clear();
        for (int i = 0; i < waypoints.rows(); ++i) {
            msg.data.push_back(waypoints(i, 0)); // x
            msg.data.push_back(waypoints(i, 1)); // y
        }
        utils.waypoints_pub.publish(msg);
        if (utils.tcp_client != nullptr) utils.tcp_client->send_waypoint(msg);
    }
    void lane_follow(double speed = -2) {
        if (speed < -1) speed = NORMAL_SPEED;
        if(pubWaypoints) {
            publish_waypoints();
        }
        double steer = utils.get_steering_angle();
        //std::cout << "lanefollowing " << steer<<" " << speed << std::endl;
        if (check_crosswalk() > 0) {
            speed *= cw_speed_ratio;
        }
        if (check_highway() > 0) {
            speed *= hw_speed_ratio;
        }
        utils.publish_cmd_vel(steer, speed);
    }
    void orientation_follow(double orientation, double speed = -2) {
        if (speed < -1) speed = NORMAL_SPEED;
        if(pubWaypoints) {
            publish_waypoints();
        }
        double yaw_error = orientation - utils.get_yaw();
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        double steer = - yaw_error * 180 / M_PI * 1;
        if (check_crosswalk() > 0) {
            speed *= cw_speed_ratio;
        }
        if (check_highway() > 0) {
            speed *= hw_speed_ratio;
        }
        utils.publish_cmd_vel(steer, speed);
    }
    void check_car() {
        double dist;
        std::list<int> cars = utils.recent_car_indices;
        // std::cout << "number of cars detected: " << cars.size() << std::endl;
        utils.debug("CHECK_CAR(): number of cars detected: " + helper::d2str(cars.size()), 5);
        int car_index = utils.object_index(OBJECT::CAR);
        if(car_index >= 0) { // if car detected
        // for (int car_index: cars) {
            utils.update_states(x_current);
            update_mpc_states(x_current[0], x_current[1], x_current[2]);
            utils.debug("CHECK_CAR(): current state: " + helper::d2str(x_current[0]) + ", " + helper::d2str(x_current[1]) + ", " + helper::d2str(x_current[2]), 4);
            int closest_idx = path_manager.find_closest_waypoint(x_current);
            dist = utils.object_distance(car_index); // compute distance to back of car
            utils.debug("CHECK_CAR(): detected car at a distance of: " + helper::d2str(dist) + ", closest index: " + helper::d2str(closest_idx) + ", end index: " + helper::d2str(path_manager.overtake_end_index), 4);
            double safety_dist = 0.3; // meters
            if (dist < MAX_CAR_DIST && dist > 0 && closest_idx >= path_manager.overtake_end_index + safety_dist * path_manager.density) { // if car is within range and ahead of ego car
                utils.object_box(car_index, bbox);
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                auto car_pose = utils.estimate_object_pose2d(x, y, yaw, bbox, dist);
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
                static double lane_offset = LANE_OFFSET * change_lane_offset_scaler ;
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
                utils.debug("CHECK_CAR(): min_dist: " + helper::d2str(min_dist) + ", min_dist_adj: " + helper::d2str(min_dist_adj), 4);
                if (on_highway && min_dist > LANE_OFFSET - CAR_WIDTH && min_dist_adj > LANE_OFFSET - CAR_WIDTH) {
                        detected_car_state = DETECTED_CAR_STATE::OPPOSITE_LANE;
                } else if (min_dist < LANE_OFFSET - CAR_WIDTH + SAME_LANE_SAFETY_FACTOR && min_dist < min_dist_adj) {
                    detected_car_state = DETECTED_CAR_STATE::SAME_LANE;
                } else if (min_dist_adj < LANE_OFFSET - CAR_WIDTH && min_dist_adj < min_dist) {
                    detected_car_state = DETECTED_CAR_STATE::ADJACENT_LANE;
                } else {
                    detected_car_state = DETECTED_CAR_STATE::NOT_SURE;
                }
                // utils.debug("CHECK_CAR(): closest waypoint to detected car: " + helper::d2str(min_index) + ", at " + helper::d2str(path_manager.state_refs(min_index, 0)) + ", " + helper::d2str(path_manager.state_refs(min_index, 1)), 3);
                // utils.debug("CHECK_CAR(): min dist between car and closest waypoint: " + helper::d2str(min_dist) + ", same lane: " + helper::d2str(detected_car_state == DETECTED_CAR_STATE::SAME_LANE), 3);
                if (detected_car_state == DETECTED_CAR_STATE::SAME_LANE) {
                    if (idx < path_manager.state_refs.rows() && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::DOTTED) && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::DOTTED_CROSSWALK) && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::HIGHWAYLEFT) && !path_manager.attribute_cmp(idx, path_manager.ATTRIBUTE::HIGHWAYRIGHT)) {
                        if (dist < MAX_TAILING_DIST) {
                            mpc.reset_solver();
                            utils.debug("CHECK_CAR(): SAME_LANE: detetected car is in one way or non-dotted region, dist = " + helper::d2str(dist) + ", stopping...", 2);
                            stop_for(20*T);
                            return;
                        } else {
                            utils.debug("CHECK_CAR(): SAME_LANE: car on oneway pretty far and within safety margin, keep tailing: " + helper::d2str(dist), 2);
                        }
                    } else { // if detected car is in dotted region or on highway, we can overtake
                        int start_index = closest_idx + static_cast<int>(start_dist * density);
                        if (start_index >= path_manager.state_refs.rows() || path_manager.overtake_end_index >= path_manager.state_refs.rows()) {
                            utils.debug("CHECK_CAR(): WARNING: start or end index exceeds state_refs size, stopping...", 2);
                            return;
                        };
                        path_manager.overtake_end_index = start_index + static_cast<int>((CAR_LENGTH * 2 + MIN_DIST_TO_CAR * 2) * density * path_manager.overtake_end_index_scaler);
                        utils.debug("CHECK_CAR(): SAME_LANE: OVERTAKING: start idx: " + helper::d2str(start_index) + ", end idx: " + helper::d2str(path_manager.overtake_end_index) + ", min_dist: " + helper::d2str(min_dist) + ", min_dist_adj: " + helper::d2str(min_dist_adj), 2);
                        path_manager.change_lane(start_index, path_manager.overtake_end_index, right, lane_offset);
                        utils.debug("CHECK_CAR(): SAME_LANE: OVERTAKING: changing lane to the " + std::string(right ? "right" : "left") + " in " + helper::d2str(start_dist) + " meters. start pose: (" + helper::d2str(path_manager.state_refs(start_index, 0)) + "," + helper::d2str(path_manager.state_refs(start_index, 1)) + "), end: (" + helper::d2str(path_manager.state_refs(path_manager.overtake_end_index, 0)) + ", " + helper::d2str(path_manager.state_refs(path_manager.overtake_end_index, 1)) + "), cur: (" + helper::d2str(x) + ", " + helper::d2str(y) + ")", 2);
                        return;
                    }
                } else if (detected_car_state == DETECTED_CAR_STATE::NOT_SURE) {
                    if (dist < MAX_TAILING_DIST) {
                        utils.debug("CHECK_CAR(): NOT_SURE: dist = " + helper::d2str(dist) + ", stopping... min_dist: " + helper::d2str(min_dist) + ", min_dist_adj: " + helper::d2str(min_dist_adj) + ", car pose: (" + helper::d2str(car_pose[0]) + ", " + helper::d2str(car_pose[1]) + ")", 2);
                        stop_for(20*T);
                        return;
                    } else {
                        utils.debug("CHECK_CAR(): NOT_SURE: car pretty far and within safety margin, keep tailing: " + helper::d2str(dist), 2);
                    }
                } else if (detected_car_state == DETECTED_CAR_STATE::OPPOSITE_LANE) {
                    return;
                }
            }
        }
    }
    bool goto_command_callback(utils::goto_command::Request &req, utils::goto_command::Response &res) {
        utils.update_states(x_current);
        if (!path_manager.call_go_to_service(x_current[0], x_current[1], x_current[2], req.dest_x, req.dest_y)) {
            res.success = false;
            return false;
        }
        auto state_refs = path_manager.state_refs.transpose();
        auto input_refs = path_manager.input_refs.transpose();
        auto& state_attributes = path_manager.state_attributes;
        auto normals = path_manager.normals.transpose();
        res.state_refs.data = std::vector<float>(state_refs.data(), state_refs.data() + state_refs.size());
        res.input_refs.data = std::vector<float>(input_refs.data(), input_refs.data() + input_refs.size());
        res.wp_attributes.data = std::vector<float>(state_attributes.data(), state_attributes.data() + state_attributes.size());
        res.wp_normals.data = std::vector<float>(normals.data(), normals.data() + normals.size());
        res.success = true;
        destination = path_manager.state_refs.row(path_manager.state_refs.rows()-1).head(2);
        // for (int i = 0; i<path_manager.state_refs.rows(); i++) {
        //     std::cout << i << ") " << path_manager.state_refs(i, 0) << ", " << path_manager.state_refs(i, 1) << ", " << path_manager.state_refs(i, 2) << std::endl;
        // }
        utils.debug("goto_command_callback(): start: " + helper::d2str(x_current(0)) + ", " + helper::d2str(x_current(1)), 2);
        utils.debug("goto_command_callback(): destination: " + helper::d2str(destination(0)) + ", " + helper::d2str(destination(1)), 2);

        path_manager.target_waypoint_index = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows()-1); // search from the beginning to the end
        path_manager.overtake_end_index = 0;
        mpc.reset_solver();
        initialized = true;
        return true;
    }

    bool goto_multiple_command_callback(std::vector<std::tuple<float, float>> &coords, utils::goto_command::Response &res) {
        utils.update_states(x_current);
        if (!path_manager.call_go_to_multiple_service(x_current[0], x_current[1], x_current[2], coords)) {
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
    
    bool set_states_callback(utils::set_states::Request &req, utils::set_states::Response &res) {
        if (req.x >= 0 && req.y >= 0) {
            utils.set_states(req.x, req.y);
        } else {
            utils.reset_yaw();
        }
        res.success = true;
        return true;
    }
};

void StateMachine::update_mpc_states() {
    utils.update_states(x_current);
    double yaw = x_current(2);
    if(path_manager.closest_waypoint_index < path_manager.state_refs.rows() && path_manager.state_refs.rows() > 0) {
        double ref_yaw = path_manager.state_refs(path_manager.closest_waypoint_index, 2);
        yaw = Utility::yaw_mod(yaw, ref_yaw);
    }
    x_current(2) = yaw;
}
void StateMachine::solve() {
    int success = path_manager.find_next_waypoint(path_manager.target_waypoint_index, x_current);
    // std::cout << "current state: x: " << x_current(0) << ", y: " << x_current(1) << ", yaw: " << x_current(2) << std::endl;
    // std::cout << "closest waypoint index: " << path_manager.closest_waypoint_index << ", at x: " << path_manager.state_refs(path_manager.closest_waypoint_index, 0) << ", y: " << path_manager.state_refs(path_manager.closest_waypoint_index, 1) << ", yaw: " << path_manager.state_refs(path_manager.closest_waypoint_index, 2) << std::endl;
    // std::cout << "target waypoint index: " << path_manager.target_waypoint_index << ", at x: " << path_manager.state_refs(path_manager.target_waypoint_index, 0) << ", y: " << path_manager.state_refs(path_manager.target_waypoint_index, 1) << ", yaw: " << path_manager.state_refs(path_manager.target_waypoint_index, 2) << std::endl;
    // for (int i = path_manager.target_waypoint_index; i < std::min(path_manager.target_waypoint_index + 6, static_cast<int>(path_manager.state_refs.rows())); i++) {
    //     std::cout << "i: " << i << ", x: " << path_manager.state_refs(i, 0) << ", y: " << path_manager.state_refs(i, 1) << ", yaw: " << path_manager.state_refs(i, 2) << std::endl;
    // }
    int idx = path_manager.target_waypoint_index;
    if (idx > path_manager.state_refs.rows()-2) {
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
void StateMachine::publish_commands() {
    static int count = 0;
    if(pubWaypoints) {
        publish_waypoints();
    }
    double steer = -mpc.u_current[1] * 180 / M_PI;
    double speed = mpc.u_current[0];
    // steer = 0;
    // speed = 0.32;
    
    // std::cout << "steer: " << steer << ", speed: " << speed << std::endl;
    // std::cout << speed*100 << ", " << steer << std::endl;
    utils.publish_cmd_vel(steer, speed);
}
void StateMachine::change_state(STATE new_state) {
    // std::cout << "Changing from " << state_names[state] << " to " << state_names[new_state] << std::endl;
    utils.debug("Changing from " + state_names[state] + " to " + state_names[new_state], 2);
    state = new_state;
}

void StateMachine::run() {
    static ros::Time overtake_cd = ros::Time::now();
    static bool wrong_lane = false;
    std::cout << "State Machine running..." << std::endl;
    while (ros::ok()) {
        utils.update_states(x_current);
        if (sign) {
            pedestrian_detected();
            exit_detected();
            check_emergency_stop();
        }
        if (state == STATE::MOVING) {
            update_mpc_states();
            solve();
            if(intersection_reached()) {
                if(stopsign_flag == OBJECT::STOPSIGN || stopsign_flag == OBJECT::LIGHTS) {
                    // change_state(STATE::WAITING_FOR_STOPSIGN);
                    if (stopsign_flag == OBJECT::STOPSIGN) {
                        utils.debug("intersection reached: CASE STOP SIGN, stopping for " + helper::d2str(stop_duration) + " seconds...", 2);
                        mpc.reset_solver();
                        stop_for(stop_duration);
                    } else if (stopsign_flag == OBJECT::LIGHTS) {
                        utils.debug("intersection reached: CASE TRAFFIC LIGHT, clearing stopsign flag...", 2);
                    }
                    stopsign_flag = OBJECT::NONE;
                } else if(stopsign_flag == OBJECT::PRIORITY) {
                    utils.debug("intersection reached: CASE PRIORITY, proceeding...", 2);
                    stopsign_flag = OBJECT::NONE;
                } else if(stopsign_flag == OBJECT::CROSSWALK) {
                    utils.debug("intersection reached: CASE CROSSWALK, proceeding...", 2);
                    stopsign_flag = OBJECT::NONE;
                } else if(stopsign_flag == OBJECT::ROUNDABOUT) {
                    utils.debug("intersection reached: CASE ROUNDABOUT, proceeding...", 2);
                    stopsign_flag = OBJECT::NONE;
                } else {
                    ROS_WARN("intersection reached: CASE NO SIGN, proceeding...");
                    stopsign_flag = OBJECT::NONE;
                }
            }
            if (sign) {
                check_stop_sign();
                check_light();
                check_highway_signs();
                int park_index = park_sign_detected();
                if(park_index>=0 && park_count < 1) {
                    auto x1 = PARKING_SIGN_POSES[0][0];
                    auto y1 = PARKING_SIGN_POSES[0][1];
                    double distance_to_parking_spot = std::sqrt(std::pow((x_current[0] - x1), 2) + std::pow((x_current[1] - y1), 2));
                    double detected_dist = utils.object_distance(park_index);
                    double abs_error = std::abs(detected_dist - distance_to_parking_spot);
                    if (abs_error < 1.) {
                        // utils.debug("parking sign detected, proceeding to parking...", 3);
                        change_state(STATE::PARKING);
                        park_count++;
                        continue;
                    } else {
                        utils.debug("WARNING: parking sign detected, but detected distance too large: " + helper::d2str(detected_dist) + ", expected: " + helper::d2str(distance_to_parking_spot) + ", relocalizing...", 2);
                    }
                }
                check_car();    
            }
            if (lane_relocalize) {
                lane_based_relocalization();
            }
            double error_sq = (x_current.head(2) - destination).squaredNorm();
            if (error_sq < TOLERANCE_SQUARED && path_manager.target_waypoint_index >= path_manager.state_refs.rows() * 0.9) {
                change_state(STATE::DONE);
            }
            rate->sleep();
            continue;
        } else if (state == STATE::APPROACHING_INTERSECTION) {
            std::cout << "approaching intersection" << std::endl;
        } else if (state == STATE::LANE_FOLLOWING) {
            std::cout << "lane following" << std::endl;
        } else if (state == STATE::WAITING_FOR_STOPSIGN) {
            stop_for(stop_duration);
            change_state(STATE::MOVING);
        } else if (state == STATE::WAITING_FOR_LIGHT) {
            stop_for(stop_duration);
            // if(lane) change_state(STATE::LANE_FOLLOWING);
            // else change_state(STATE::MOVING);
            change_state(STATE::MOVING);
        } else if (state == STATE::PARKING) {
            stop_for(stop_duration/2);
            double offset_thresh = 0.1;
            double base_offset = detected_dist + PARKING_SPOT_LENGTH * 1.5 + offset_thresh;
            double offset = base_offset;
            utils.debug("park sign detected at a distance of: " + helper::d2str(detected_dist) + ", parking offset is: " + helper::d2str(offset), 2);
            // base_offset = 0;
            right_park = true;
            bool hard_code = true;
            int target_spot = 0;
            auto temp_rate = ros::Rate(50);
            if (true) {
                double orientation = Utility::nearest_direction(utils.get_yaw());
                ROS_INFO("orientation: %.3f", orientation);
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                int park_index = utils.object_index(OBJECT::PARK);
                if(park_index >= 0) {
                    double dist = utils.object_distance(park_index);
                } else {
                    ROS_WARN("parking sign invalid... returning to STATE::MOVING");
                    change_state(STATE::MOVING);
                }
                // if (1) {
                if (sign_relocalize) {
                    auto park_sign_pose = utils.estimate_object_pose2d(x0, y0, yaw0, utils.object_box(park_index), detected_dist);
                    int success = sign_based_relocalization(park_sign_pose, PARKING_SIGN_POSES, "PARKING");
                }
                utils.debug("PARKING(): target spot: " + helper::d2str(target_spot), 2);
                for(int i = 0; i<PARKING_SPOTS.size(); i++) {
                    std::cout << "parking spot " << i << ": " << PARKING_SPOTS[i][0] << ", " << PARKING_SPOTS[i][1] << std::endl;
                }
                while(1) {
                    pedestrian_detected();
                    exit_detected();
                    check_emergency_stop();
                    // check utils.recent_car_indices
                    // std::cout << "recent car indices size: " << utils.recent_car_indices.size() << std::endl;
                    std::list<int> cars = utils.recent_car_indices;
                    // std::cout << "number of cars detected: " << cars.size() << std::endl;
                    // iterate through all cars and check if any are in the parking spot
                    bool changed = false;
                    bool car_in_spot = false;
                    while(1) {
                        for (int i : cars) {
                            utils.debug("PARKING(): checking car: (" + helper::d2str(utils.detected_cars[i][0]) + ", " + helper::d2str(utils.detected_cars[i][1]) + "), error: " + helper::d2str((utils.detected_cars[i] - PARKING_SPOTS[target_spot]).norm()), 5);
                            Eigen::Vector2d world_pose = utils.detected_cars[i];
                            Eigen::Vector2d spot = PARKING_SPOTS[target_spot];
                            double error_sq = (world_pose - spot).squaredNorm();
                            double error_threshold_sq = 0.04;
                            if (error_sq < error_threshold_sq) {
                                car_in_spot = true;
                                utils.debug("PARKING(): car detected in spot: " + helper::d2str(target_spot) + ", error: " + helper::d2str(std::sqrt(error_sq)), 2);
                                target_spot++;
                                changed = true;
                                break;
                            }
                            car_in_spot = false;
                        }
                        if (!car_in_spot) break;
                    }
                    if (changed) {
                        offset = base_offset + target_spot / 2 * PARKING_SPOT_LENGTH;
                        right_park = target_spot % 2 == 0;
                        ROS_INFO("car in spot, changing to target spot %d at (%.3f, %.3f), right: %s", target_spot, PARKING_SPOTS[target_spot][0], PARKING_SPOTS[target_spot][1], right_park ? "true" : "false");
                    }
                    double x, y, yaw;
                    utils.get_states(x, y, yaw);
                    double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                    if (norm_sq >= offset * offset)
                    {
                        ROS_INFO("x offset reached: (%.2f, %.2f), ready for parking maneuver...", x, y);
                        stop_for(stop_duration/2);
                        break;
                    }
                    orientation_follow(orientation);
                    // solve();
                    // if (norm_sq > offset * offset * 0.2) {
                    //     orientation_follow(orientation);
                    // } else {
                    //     update_mpc_states();
                    //     solve();
                    // }
                    temp_rate.sleep();
                }
            }
            stop_for(stop_duration/2);
            if (hard_code) {
                // right_park = true; //temp
                double orientation = Utility::nearest_direction(utils.get_yaw());
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double initial_y_error = y - (PARKING_SPOTS[target_spot][1] + PARKING_SPOT_WIDTH * (right_park ? 1 : -1));
                double initial_yaw_error = orientation - yaw;
                initial_yaw_error = Utility::yaw_mod(initial_yaw_error); // normalize to [-pi, pi]
                // ROS_INFO("initial y error: %.3f, initial yaw error: %.3f", initial_y_error, initial_yaw_error);
                utils.debug("orientation: " + helper::d2str(orientation) + ", yaw: " + helper::d2str(yaw), 4);
                // exit(0);
                parking_maneuver_hardcode(right_park, false, 1/T_park, initial_y_error, initial_yaw_error);
            }
            double x, y, yaw;
            utils.get_states(x, y, yaw);
            double x_error = x - PARKING_SPOTS[target_spot][0];
            if (std::abs(x_error) > 0.15) {
                double orientation = Utility::nearest_direction(utils.get_yaw());
                ROS_INFO("parked but x offset too large: %.3f, adjusting... orientation: %.3f", x_error, orientation);
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                while(1) {
                    x_error = x - PARKING_SPOTS[target_spot][0];
                    pedestrian_detected();
                    exit_detected();
                    check_emergency_stop();
                    utils.get_states(x, y, yaw);
                    double norm_sq = std::pow(x - x0, 2) + std::pow(y - y0, 2);
                    if (x_error > 0 && x_error < 0.05 || x_error < 0 && x_error > -0.05)
                    {
                        ROS_INFO("parking spot reached, stopping...");
                        utils.publish_cmd_vel(0.0, 0.0);
                        break;
                    }
                    double speed = NORMAL_SPEED;
                    if (x_error > 0) {
                        speed = -speed;
                    }
                    orientation_follow(orientation, speed);
                    temp_rate.sleep();
                }
            }
            change_state(STATE::PARKED);
        } else if (state == STATE::PARKED) {
            stop_for(stop_duration);
            change_state(STATE::EXITING_PARKING);
        } else if (state == STATE::EXITING_PARKING) {
            bool hard_code = true;
            if (hard_code) {
                // right_park = true;
                parking_maneuver_hardcode(right_park, true, 1/T_park);
            }
            utils.update_states(x_current);
            path_manager.target_waypoint_index = path_manager.find_closest_waypoint(x_current, 0, path_manager.state_refs.rows() - 1);
            std::cout << "exiting_park(): target waypoint index: " << path_manager.target_waypoint_index << ", at (" << path_manager.state_refs(path_manager.target_waypoint_index, 0) << ", " << path_manager.state_refs(path_manager.target_waypoint_index, 1) << ")" << std::endl;
            std::cout << "exiting_park(): closest waypoint index: " << path_manager.closest_waypoint_index << ", at (" << path_manager.state_refs(path_manager.closest_waypoint_index, 0) << ", " << path_manager.state_refs(path_manager.closest_waypoint_index, 1) << ")" << std::endl;
            std::cout << "exiting_park(): current state: x: " << x_current(0) << ", y: " << x_current(1) << ", yaw: " << x_current(2) << std::endl;
            change_state(STATE::MOVING);
        } else if (state == STATE::INTERSECTION_MANEUVERING) {
            std::cout << "intersection maneuvering" << std::endl;
        } else if (state == STATE::INIT) {
            // initialize();
            if (dashboard) {
                // utils.publish_cmd_vel(0, 0);
                rate->sleep();
            } else {
                initialize();
                start();
            }
        } else if (state == STATE::DONE) {
            utils.debug("Done", 5);
            utils.stop_car();
            continue;
        } else if (state == STATE::KEYBOARD_CONTROL) {
            // Constants for steering and speed
            const double STEERING_INCREMENT = 1;  
            const double VELOCITY_INCREMENT = 0.05;   
            const double MAX_VELOCITY = 1.45;     
            const double MIN_VELOCITY = -0.45;    
            const double HARD_MAX_STEERING = 25.0;
            const double STEERING_DECAY = 1.25;
            const double VELOCITY_DECAY = 0; //0.0025;
            double velocity = 0.0;
            double steering_angle = 0.0;

            // Initialize ncurses mode
            initscr();
            cbreak();
            noecho();
            keypad(stdscr, TRUE);  // Enable keyboard mapping
            timeout(100);          // Non-blocking delay to allow continuous updates

            // Information display
            printw("Use 'w' and 's' to increase or decrease speed.\n");
            printw("Use 'a' and 'd' to control steering.\n");
            printw("'q' to quit.\n");

            int ch;
            bool running = true;

            while (running) {
                ch = getch();
                
                switch (ch) {
                    case 'w':
                        velocity += VELOCITY_INCREMENT;
                        if (velocity > MAX_VELOCITY) velocity = MAX_VELOCITY;
                        break;
                    case 's':
                        velocity -= VELOCITY_INCREMENT;
                        if (velocity < MIN_VELOCITY) velocity = MIN_VELOCITY;
                        break;
                    case 'a':
                        steering_angle -= STEERING_INCREMENT;
                        if (steering_angle < -HARD_MAX_STEERING) steering_angle = -HARD_MAX_STEERING;
                        break;
                    case 'd':
                        steering_angle += STEERING_INCREMENT;
                        if (steering_angle > HARD_MAX_STEERING) steering_angle = HARD_MAX_STEERING;
                        break;
                    case 'b':
                        velocity = 0;
                        break;
                    case 'q':
                        running = false;
                        break;
                    default:
                        // Gradually return the steering towards zero if no steering keys are pressed
                        if (steering_angle > 0) {
                            steering_angle -= STEERING_DECAY;
                            if (steering_angle < 0) steering_angle = 0;
                        } else if (steering_angle < 0) {
                            steering_angle += STEERING_DECAY;
                            if (steering_angle > 0) steering_angle = 0;
                        }
                        if (velocity > 0) {
                            velocity -= VELOCITY_DECAY;
                            if (velocity < 0) velocity = 0;
                        } else if (velocity < 0) {
                            velocity += VELOCITY_DECAY;
                            if (velocity > 0) velocity = 0;
                        }
                        break;
                }

                // Clear previous outputs
                clear();
                printw("Velocity: %f\n", velocity);
                printw("Steering angle: %f\n", steering_angle);
                printw("Press 'q' to quit.");
                utils.publish_cmd_vel(steering_angle, velocity);

            }

            // Clean up ncurses
            endwin();
            utils.stop_car();
            change_state(STATE::INIT);
        } else if (state == STATE::TESTING) {
            // lane_based_relocalization();
            int sign_index = utils.object_index(OBJECT::STOPSIGN);
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::REDLIGHT);
            }
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::LIGHTS);
            }
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::GREENLIGHT);
            }
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::YELLOWLIGHT);
            }
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::PRIORITY);
            }
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::ROUNDABOUT);
            }
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::CROSSWALK);
            }
            bool is_car = false;
            if (sign_index < 0) {
                sign_index = utils.object_index(OBJECT::CAR);
                is_car = true;
            }
            if (sign_index < 0) {
                continue;
            }
            auto start = std::chrono::high_resolution_clock::now();
            double detected_dist1 = utils.object_distance(sign_index);
            std::cout << "detected_dist: " << detected_dist1 << std::endl;
            auto sign_pose1 = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], utils.object_box(sign_index), detected_dist1, is_car);
            ROS_INFO("current_pose: (%.2f, %.2f, %.2f)", x_current[0], x_current[1], x_current[2]);
            ROS_INFO("sign_pose: (%.2f, %.2f)", sign_pose1[0], sign_pose1[1]); 
            ROS_INFO("Relative pose: (%.2f, %.2f)", sign_pose1[0] - x_current[0], sign_pose1[1] - x_current[1]);
            std::string string1 = "";
            auto relevant_signs = utils.get_relevant_signs(sign_index, string1);
            sign_based_relocalization(sign_pose1, relevant_signs, string1);
            auto end = std::chrono::high_resolution_clock::now();
            std::chrono::duration<double> elapsed = end - start;
            std::cout << "elapsed time: " << elapsed.count() << "s" << std::endl;
            rate->sleep();
            continue;
        }
    }
}


StateMachine *globalStateMachinePtr = nullptr;
void signalHandler(int signum) {
    if (globalStateMachinePtr) {
        globalStateMachinePtr->utils.stop_car();
        globalStateMachinePtr->call_trigger_service();
    }
    if (globalStateMachinePtr->utils.serial && globalStateMachinePtr->utils.serial->is_open()) {
        globalStateMachinePtr->utils.serial->close();
    }
    globalStateMachinePtr->utils.serial.reset();
    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv) {

    std::cout.precision(3);
    //create anonymous node handle
    ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    double T, v_ref, T_park;
    int N;
    bool sign, ekf, lane, real, use_beta, async;
    std::string name;
    std::string nodeName = ros::this_node::getName();
    std::cout << "node name: " << nodeName << std::endl;
    bool success = nh.getParam(nodeName + "/lane", lane) && nh.getParam(nodeName+"/ekf", ekf) && nh.getParam(nodeName+"/sign", sign) && nh.getParam("T", T) && nh.getParam("N", N) && nh.getParam("constraints/v_ref", v_ref);
    double x0, y0, yaw0, vref;
    success = success && nh.getParam(nodeName+"/name", name) && nh.getParam(nodeName+"/vref", vref) && nh.getParam(nodeName+"/x0", x0) && nh.getParam(nodeName+"/y0", y0) && nh.getParam(nodeName+"/yaw0", yaw0);
    success = success && nh.getParam("/T_park", T_park);
    success = success && nh.getParam(nodeName+"/real", real);
    success = success && nh.getParam("/use_beta", use_beta) && nh.getParam("/async", async);
    if (!success) {
        std::cout << "Failed to get parameters" << std::endl;
        exit(1);
    } else {
        std::cout << "Successfully loaded parameters" << std::endl;
    }
    std::cout << "ekf: " << ekf << ", sign: " << sign << ", T: " << T << ", N: " << N << ", vref: " << vref << ", real: " << real << std::endl;
    StateMachine sm(nh, T, N, vref, sign, ekf, lane, T_park, name, x0, y0, yaw0, real, use_beta);
    bool use_tcp = false;
    nh.getParam("/use_tcp", use_tcp);

    globalStateMachinePtr = &sm;
    signal(SIGINT, signalHandler);
    
    std::thread services_thread;
    if(use_tcp) {
        services_thread = std::thread(&StateMachine::receive_services, &sm);
    } 

    std::thread callback_thread;
    std::unique_ptr<ros::AsyncSpinner> spinner;
    if (async) {
        int num_threads = 4;
        spinner = std::make_unique<ros::AsyncSpinner>(num_threads);
        spinner->start();
        std::cout << "Async spinner started with " << num_threads << " threads" << std::endl;
    } else {
        callback_thread = std::thread(&Utility::spin, &sm.utils);
    }
    
    sm.run();

    if (services_thread.joinable()) {
        services_thread.join();
    }
    ros::waitForShutdown();
    if (callback_thread.joinable()) {
        callback_thread.join();
    }
    return 0;
}
