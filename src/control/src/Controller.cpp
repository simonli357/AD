#include <memory>
#include <ostream>
#include <ros/ros.h>
#include <ros/subscriber.h>
#include <thread>
#include <map>
#include <string>
#include <tuple>
#include <vector>
#include <mutex>
#include <chrono>
#include "Database.hpp"
#include "queries/GraphQueries.hpp"
#include "std_srvs/SetBoolRequest.h"
#include "Utility.hpp"
#include "PathManager.h"
#include "MPC.hpp"
#include <signal.h>
#include <fstream>
#include <iostream>
#include "utils/constants.h"
#include "utils/waypoints.h"
#include "utils/goto_command.h"
#include "utils/set_states.h"
#include <std_srvs/Trigger.h>
#include <std_srvs/SetBool.h>
#include <ncurses.h>
#include <std_msgs/Byte.h>
#include "utils/helper.h"
#include "Tracking.h"
#include "GroundTruth.h"
#include "Tunable.h"
#include "EgoCar.h"
#include "Sensing.h"
#include <tbb/task_group.h>

using namespace VehicleConstants;
using namespace Tunable;

class StateMachine {
public:
    StateMachine(ros::NodeHandle& nh_, Database &db): 
    nh(nh_), utils(nh), mpc(Tunable::T,Tunable::N,Tunable::v_ref,Tunable::use_beta), //path_manager(nh,Tunable::T,Tunable::N,Tunable::v_ref, utils.pathName),
    state(STATE::INIT)
    {
        PathManager::init(nh, Tunable::T, Tunable::N, Tunable::v_ref, utils.pathName);
        bool success = true;
        if (keyboardControl) {
            utils.debug("keyboard control enabled", 2);
            change_state(STATE::KEYBOARD_CONTROL);
        }

        double rateVal = 1 / Tunable::T;
        rate = new ros::Rate(rateVal);
        std::cout << "rate: " << rateVal << std::endl;
        goto_command_server = nh.advertiseService("/goto_command", &StateMachine::goto_command_callback, this);
        set_states_server = nh.advertiseService("/set_states", &StateMachine::set_states_callback, this);
        utils.debug("start_bool server ready, mpc time step T = " + helper::d2str(Tunable::T), 2);
        utils.debug("state machine initialized", 2);
        db.graph_queries->set_graph(PathManager::path_planner.serialized_graph);


        model_states = nh.subscribe("/gazebo/model_states", 3, &StateMachine::model_callback, this);

        ThreadPools::communication.execute([this] {
            tcp_callbacks = std::make_unique<tbb::task_group>();
        });

        // set callbacks for tcp client
        utils.tcp_client->set_send_run_callback(
            [this]() {
                tcp_callbacks->run([this] {
                    std::cout << "Sending Run Parameters" << std::endl;
                    if (state == STATE::INIT || state == STATE::DONE) {
                        utils.tcp_client->send_start_srv(false);
                    } else {
                        utils.tcp_client->send_start_srv(true);
                    }
                    utils.fetch_run_params();
                    utils.tcp_client->send_run(PathManager::v_ref, PathManager::pathName, utils.x0, utils.y0, utils.yaw0);
                });
            }
        );

        utils.tcp_client->set_go_to_cmd_callback(
            [this](const std::vector<std::tuple<float, float>> &coords) {
                tcp_callbacks->run([this, coords] {
                    std::cout << "Planning Path" << std::endl;
                    utils::goto_command::Response res;
                    goto_multiple_command_callback(coords, res);
                    utils.tcp_client->send_go_to_cmd_srv(res.state_refs, res.input_refs, res.wp_attributes, res.wp_normals, true);
                });
            }
        );

        utils.tcp_client->set_set_states_callback(
            [this](double x, double y) {
                tcp_callbacks->run([this, x, y] {
                    std::cout << "Setting States" << std::endl;
                    utils::set_states::Request req;
                    utils::set_states::Response res;
                    req.x = x;
                    req.y = y;
                    set_states_callback(req, res);
                    utils.tcp_client->send_set_states_srv(true);
                });
            }
        );

        utils.tcp_client->set_start_callback(
            [this](bool started) {
                tcp_callbacks->run([this, started] {
                    std::cout << "Sending State" << std::endl;
                    start_bool_callback(started);
                    if (state == STATE::INIT || state == STATE::DONE) {
                        utils.tcp_client->send_start_srv(false);
                    } else {
                        utils.tcp_client->send_start_srv(true);
                    }
                });
            }
        );

        utils.tcp_client->set_ack_callback(
            [this]() {
                tcp_callbacks->run([this] {
                    std::cout << "Sending State" << std::endl;
                    if (state == STATE::INIT || state == STATE::DONE) {
                        utils.tcp_client->send_start_srv(false);
                    } else {
                        utils.tcp_client->send_start_srv(true);
                    }
                });
            }
        );

        utils.tcp_client->set_waypoints_callback(
            [this](double x0, double y0, double yaw0) {
                tcp_callbacks->run([this, x0, y0, yaw0] {
                    std::cout << "Setting Waypoints" << std::endl;
                    PathManager::call_waypoint_service(x0, y0, yaw0, utils.tcp_client);
                });
            }
        );

        utils.tcp_client->set_yaw_callback(
            [this](int direction) {
                tcp_callbacks->run([this, direction] {
                    std::cout << "Setting Yaw" << std::endl;
                    Sensing::reset_yaw(direction);
                });
            }
        );

        initialize();
    }
    ~StateMachine() {
        // utils.stop_car();
        tcp_callbacks->wait();
    }
    ros::NodeHandle& nh;
    ros::Subscriber model_states;

    std::unique_ptr<tbb::task_group> tcp_callbacks;

    bool initialized = false;
    bool wait_for_green_flag = false;

    Eigen::Vector3d x_current;

    std::array<double, 4> bbox = {0.0, 0.0, 0.0, 0.0};
    double detected_dist = 0;
    bool right_park = true;
    int park_count = 0;
    OBJECT sign_flag = OBJECT::NONE;
    ros::Time sign_cooldown_timer = ros::Time::now();
    ros::Time highway_cooldown_timer = ros::Time::now();
    Eigen::Vector2d destination;
    int state = 0;
    
    ros::Rate* rate;

    std::mutex lock;
    Utility utils;
    // PathManager path_manager;
    ros::ServiceServer goto_command_server, set_states_server;
    MPC mpc;

    // intersection variables
    Eigen::Vector2d last_intersection_point = {1000.0, 1000.0};

    std::optional<size_t> car_idx;

    void call_trigger_service() {
        ros::ServiceClient client = nh.serviceClient<std_srvs::Trigger>("/trigger_service");
        std_srvs::Trigger srv;
        client.call(srv);
    }
    void model_callback(const gazebo_msgs::ModelStates::ConstPtr& msg) {
        if (!car_idx.has_value()) {
            auto it = std::find(msg->name.begin(), msg->name.end(), robot_name);
            if (it != msg->name.end()) {
                car_idx = std::distance(msg->name.begin(), it);
                std::cout << "automobile found: " << *car_idx << std::endl;
            } else {
                printf("automobile not found\n");
                return; 
            }
        }
        utils.tcp_client->send_model_states(msg->pose[*car_idx]);
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
        PathManager::call_waypoint_service(x, y, yaw, utils.tcp_client);
        destination = PathManager::state_refs.row(PathManager::state_refs.rows()-1).head(2);
        utils.debug("INITIALIZE(): start: " + helper::d2str(x) + ", " + helper::d2str(y), 2);
        utils.debug("INITIALIZE(): destination: " + helper::d2str(destination(0)) + ", " + helper::d2str(destination(1)), 2);

        // std::cout << "INITIALIZE(): target waypoint index: " << PathManager::target_waypoint_index << std::endl;
        PathManager::target_waypoint_index = PathManager::find_closest_waypoint(x_current); // search from the beginning to the end
        // std::cout << "INITIALIZE(): target waypoint index2: " << PathManager::target_waypoint_index << std::endl;
        PathManager::target_waypoint_index = 0;
        PathManager::find_intersections(utils);
        mpc.reset_solver();
        initialized = true;
        
        // send initial state of car
        if (state == STATE::INIT || state == STATE::DONE) {
            utils.tcp_client->send_start_srv(false);
        } else {
            utils.tcp_client->send_start_srv(true);
        }
        return 1;
    }
    int start() {
        if (testing) {
            change_state(STATE::TESTING);
        } else {
            change_state(STATE::MOVING);
        }
        return 1;
    }
    bool start_bool_callback(bool started) {
        static int history = -1;
        if (started && (state == STATE::INIT || state == STATE::DONE)) {
            start();
        } else {
            history = state;
            stop_for(10*T);
            change_state(STATE::INIT);
            stop_for(10*T);
        }
        return true;
        
    }
    void solve();
    void publish_commands();
    void update_mpc_states();
    void update_mpc_states(double x, double y, double yaw) {
        if(PathManager::closest_waypoint_index < PathManager::state_refs.rows() && PathManager::state_refs.rows() > 0) {
            double ref_yaw = PathManager::state_refs(PathManager::closest_waypoint_index, 2);
            yaw = helper::yaw_mod(yaw, ref_yaw);
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
    int maneuver_hardcode(const Eigen::VectorXd &targets, const Eigen::VectorXd &steerings, const Eigen::VectorXd &speeds, const Eigen::VectorXd &thresholds, double rate_val = 20) {
        /* 
        targets: target yaws or distances
        thresholds: thresholds for each target        
        */
        double x0, y0, yaw0;
        // get current states
        utils.get_states(x0, y0, yaw0);
        // get closest direction
        yaw0 = helper::nearest_direction(yaw0);
        yaw0 = helper::yaw_mod(yaw0);
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
            yaw = helper::yaw_mod(yaw);
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

    bool check_intersection() {
        int idx = PathManager::intersection_index;
        if (idx >= PathManager::intersection_indices.size()) {
            return false;
        }
        auto& target_intersection = GroundTruth::intersections_all[PathManager::intersection_indices[PathManager::intersection_index]];
        
        if (target_intersection.associated_sign) {
            sign_flag = target_intersection.associated_sign->type;
            if (sign_flag == OBJECT::NONE) {
                Eigen::Vector3d& target_sign_pose = target_intersection.associated_sign->pose;
                for (auto& known_static_object: Tracking::get_road_known_static_objects()) {
                    Eigen::Vector3d& static_object_gt_pose = known_static_object->gt_pose;
                    double error_sq = (target_sign_pose - static_object_gt_pose).squaredNorm();
                    if (error_sq < 0.01) {
                        if (known_static_object->cumulative_confidence < 2.0) {
                            // utils.debug("CHECK_INTERSECTION(): WARNING: sign found for intersection at (" + helper::d2str(target_intersection.pose[0]) + ", " + helper::d2str(target_intersection.pose[1]) + "), but cumulative confidence too low: " + helper::d2str(known_static_object->cumulative_confidence) + ", sign: " + OBJECT_NAMES[sign_flag] + ", error: " + helper::d2str(std::sqrt(error_sq)), 2);
                            continue;
                        }
                        sign_flag = known_static_object->type;
                        target_intersection.associated_sign->type = sign_flag;
                        utils.debug("CHECK_INTERSECTION(): sign found for intersection at (" + helper::d2str(target_intersection.pose[0]) + ", " + helper::d2str(target_intersection.pose[1]) + "), sign: " + OBJECT_NAMES[sign_flag] + ", cumulative confidence: " + helper::d2str(known_static_object->cumulative_confidence) + ", error: " + helper::d2str(std::sqrt(error_sq)), 2);
                        break;
                    }
                }
            }
        } else {
            std::cerr << "CHECK_INTERSECTION(): ERROR: no associated sign for intersection " << PathManager::intersection_indices[PathManager::intersection_index] << ", index: " << PathManager::intersection_index << std::endl;
            assert(false);
        }

        if (idx > PathManager::intersection_indices.size() || idx < 0) {
            utils.debug("CHECK_INTERSECTION(): FATAL ERROR: intersection index out of bounds, idx: " + std::to_string(idx) + ", size: " + std::to_string(PathManager::intersection_indices.size()), 2);
            stop_for(10*T);
            exit(1);
        }

        // check by waypoint index. considered missed if current target exceed by 1.0meter
        int target_state_ref_idx = PathManager::intersection_state_refs_indices[idx];
        int current_target_idx = PathManager::target_waypoint_index;
        if (current_target_idx >= target_state_ref_idx + static_cast<int>(PathManager::density * 1)) {
            utils.debug("CHECK_INTERSECTION(): ERROR: INTERSECTION MISSED: target state ref index: " + std::to_string(target_state_ref_idx) + ", current target index: " + std::to_string(current_target_idx) +", incrementing idx to " + std::to_string(idx+1), 1);
            PathManager::intersection_index++;
            return false;
        }

        int intersection_id = PathManager::intersection_indices[idx];
        const auto& next_intersection_pose = GroundTruth::intersections_all[intersection_id].pose;
        double distance_to_next_sq = (x_current.head(2) - next_intersection_pose.head(2)).squaredNorm();
        
        if (distance_to_next_sq < constant_distance_to_intersection_at_detection * constant_distance_to_intersection_at_detection) {
            double yaw = x_current[2];
            double yaw_error = helper::compare_yaw(next_intersection_pose[2], yaw);
            if (yaw_error > 30 * M_PI / 180) {
                // utils.debug("CHECK_INTERSECTION(): FAILURE: yaw error too large: current: " + helper::d2str(yaw) + ", target: " + helper::d2str(next_intersection_pose[2]) + ", error: " + helper::d2str(yaw_error), 2);
                return false;
            }
            last_intersection_point = {x_current[0], x_current[1]};
            PathManager::intersection_index++;
            if (PathManager::intersection_index < PathManager::intersection_indices.size()) {
                int next_idx = PathManager::intersection_indices[PathManager::intersection_index];
                const auto& next_pose = GroundTruth::intersections_all[next_idx].pose;
    
                utils.debug("CHECK_INTERSECTION(" + OBJECT_NAMES[sign_flag] +
                            "): REACHED: x_cur: (" +
                            helper::d2str(x_current[0]) + ", " + helper::d2str(x_current[1]) + ", " + helper::d2str(x_current[2]) +
                            "), intersection: (" + helper::d2str(next_intersection_pose[0]) + ", " + helper::d2str(next_intersection_pose[1]) +
                            "), distance: " + helper::d2str(std::sqrt(distance_to_next_sq)) +
                            ", index: " + std::to_string(PathManager::intersection_index) +
                            ", next intersection: (" + helper::d2str(next_pose[0]) + ", " + helper::d2str(next_pose[1]) + ", " + helper::d2str(next_pose[2]) + ")", 2);
            } else {
                utils.debug("CHECK_INTERSECTION(" + OBJECT_NAMES[sign_flag] +
                            "): REACHED: x_cur: (" +
                            helper::d2str(x_current[0]) + ", " + helper::d2str(x_current[1]) + ", " + helper::d2str(x_current[2]) +
                            "), intersection: (" + helper::d2str(next_intersection_pose[0]) + ", " + helper::d2str(next_intersection_pose[1]) +
                            "), distance: " + helper::d2str(std::sqrt(distance_to_next_sq)) +
                            ", index: " + std::to_string(PathManager::intersection_index) +
                            ", no more intersections", 2);
            }
            if(sign_flag == OBJECT::STOPSIGN) {
                utils.debug("intersection reached: CASE STOP SIGN, stopping for " + helper::d2str(stop_duration) + " seconds...", 2);
                mpc.reset_solver();
                stop_for(stop_duration);
            } else if(sign_flag == OBJECT::NONE) {
                utils.debug("intersection reached: WARNING: CASE NO SIGN, stopping for safety...", 2);
                mpc.reset_solver();
                stop_for(stop_duration * 0.75);
            } else {
                utils.debug("intersection reached: CASE " + OBJECT_NAMES[sign_flag] + ", proceeding...", 2);
            }
            sign_flag = OBJECT::NONE;
            return true;
        } else {
            return false;
        }
    }
    bool sign_in_path(int sign_idx, double search_dist) {
        auto estimated_sign_pose = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], 
            utils.object_box(sign_idx), utils.object_distance(sign_idx));
        double x = estimated_sign_pose[0];
        double y = estimated_sign_pose[1];
        int closest_idx = PathManager::closest_waypoint_index;
        int num_index = static_cast<int>(search_dist * PathManager::density);
        double min_dist_sq = std::numeric_limits<double>::max();
        double threshold = INTERSECTION_TO_SIGN * INTERSECTION_TO_SIGN * 1.5 * 1.5;
        for (int i = closest_idx; i < closest_idx + num_index; i+=4) {
            if (i >= PathManager::state_refs.rows()) break;
            double dist_sq = std::pow(x - PathManager::state_refs(i, 0), 2) + std::pow(y - PathManager::state_refs(i, 1), 2);
            if (dist_sq < min_dist_sq) {
                min_dist_sq = dist_sq;
            }
            if (min_dist_sq < threshold) {
                return true;
            }
        }
        return false;
    }
    void check_light() {
        // check if next intersection's associated sign is a light
        // if so check if last detection time is recent
        // if so check if distance within bounds
        // if so check if light is red
        // if so stop and wait for green
        if (sign_flag == OBJECT::NONE) wait_for_green_flag = false;
        if (sign_flag != OBJECT::LIGHTS) {
            return; // if next intersection's associated sign is not a light, return
        }
        if (wait_for_green_flag) {
            return; // already waited for green
        }
        for(auto& obj: Tracking::road_known_static_objects) {
            if (obj->type != OBJECT::LIGHTS) continue;
            auto light_obj = std::dynamic_pointer_cast<Tracking::LightObject>(obj);
            if (!light_obj) {
                utils.debug("check_light(): ERROR: object has type LIGHTS but is not a LightObject", 2);
                continue;
            }
            if (light_obj->last_detection_time < ros::Time::now() - ros::Duration(0.5)) continue;
            double dist = (light_obj->gt_pose.head(2) - x_current.head(2)).norm();
            if (dist > max_light_dist) continue;
            if (light_obj->current_color == LightColor::RED) {
                wait_for_green_flag = true;
                utils.debug("check_light(): red light detected at a distance of: " + helper::d2str(dist) + ", waiting for green...", 2);
                ros::Time start_wait_time = ros::Time::now();
                while (ros::Time::now() - start_wait_time < ros::Duration(3.0)) {
                    if (light_obj->current_color == LightColor::GREEN || light_obj->current_color == LightColor::YELLOW) {
                        utils.debug("check_light(): green light detected, proceeding...", 2);
                        break;
                    }
                    stop_for(T);
                }
                return;
            }
        }
    }
    void find_sign_for_relocalization() {
        if (!sign_relocalize) return;
        // check cooldown
        if (sign_cooldown_timer > ros::Time::now()) return;
        // check sign flag
        if (sign_flag == OBJECT::NONE) {
            return;
        }
        GroundTruth::Intersection& target_intersection = GroundTruth::intersections_all[PathManager::intersection_indices[PathManager::intersection_index]];
        if (!target_intersection.associated_sign) return;
        Eigen::Vector3d& target_sign_pose = target_intersection.associated_sign->pose;
        auto known_static_objects = Tracking::get_road_known_static_objects();
        for (auto& obj: known_static_objects) {
            // check type
            if (obj->type != sign_flag) continue;
            // check last detection time
            if (obj->last_detection_time < ros::Time::now() - ros::Duration(Tunable::recency_thresholds[static_cast<int>(sign_flag)])) continue;
            // check confidence
            if (obj->cumulative_confidence < obj->cumulative_confidence_thresh) continue;
            double dist = (obj->gt_pose.head(2) - x_current.head(2)).norm();
            // check distance
            if (dist > max_sign_dist || dist < min_sign_dist) continue;
            // check error against target sign gt
            double error_sq = (target_sign_pose.head(2) - obj->gt_pose.head(2)).squaredNorm();
            if (error_sq > 0.01) continue;
            sign_cooldown_timer = ros::Time::now() + ros::Duration(Tunable::sign_cooldown);
            if (sign_based_relocalization2(obj)) obj->reset();
            return;
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
    void find_highway_for_relocalization() {
        // TODO: move magic numbers to tunable
        if (!sign_relocalize) return;
        if(highway_cooldown_timer > ros::Time::now()) {
            return;
        }
        auto known_static_objects = Tracking::get_road_known_static_objects();
        for (auto& obj: known_static_objects) {
            // check type
            if (obj->type != OBJECT::HIGHWAYEXIT && obj->type != OBJECT::HIGHWAYENTRANCE) continue;
            // check last detection time
            if (obj->last_detection_time < ros::Time::now() - ros::Duration(Tunable::recency_thresholds[static_cast<int>(obj->type)])) {
                continue;
            }
            // check confidence
            if (obj->cumulative_confidence < obj->cumulative_confidence_thresh) {
                continue;
            }
            double dist = (obj->gt_pose.head(2) - x_current.head(2)).norm();
            // check distance
            if (dist > max_sign_dist * 2 || dist < min_sign_dist) {
                continue;
            }
            highway_cooldown_timer = ros::Time::now() + ros::Duration(Tunable::highway_cooldown);
            if (sign_based_relocalization2(obj)) obj->reset();
            return;
        }
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
    void pedestrian_detected2() {
        int closest_idx = PathManager::closest_waypoint_index;
        int num_index = static_cast<int>(Tunable::pedestrian_distance * PathManager::density);
        bool pedestrian_previously_detected = false;
        while (true) {
            auto pedestrians = Tracking::get_road_pedestrians();
            if (pedestrians.empty()) {
                if (pedestrian_previously_detected) {
                    utils.debug("pedestrian_detected2(): All pedestrians disappeared.", 2);
                    pedestrian_previously_detected = false;
                }
                break;
            }
            bool detected = false;
            for (const auto& pedestrian : pedestrians) {
                if (pedestrian->type != OBJECT::PEDESTRIAN) continue;
    
                if (pedestrian->cumulative_confidence < 
                    Tunable::cumulative_confidence_thresholds[static_cast<int>(OBJECT::PEDESTRIAN)]) {
                    continue;
                }
    
                Eigen::Vector2d pedestrian_pose(pedestrian->x, pedestrian->y);
                int pedestrian_idx = PathManager::find_closest_waypoint2(
                    pedestrian_pose, Tunable::pedestrian_threshold, closest_idx, closest_idx + num_index);
    
                if (pedestrian_idx >= 0) {
                    detected = true;
                    break;
                }
            }
    
            if (detected) {
                if (!pedestrian_previously_detected) {
                    utils.debug("pedestrian_detected2(): Detected nearby pedestrian — stopping.", 2);
                    pedestrian_previously_detected = true;
                }
                stop_for(stop_duration / 5);
            } else {
                if (pedestrian_previously_detected) {
                    utils.debug("pedestrian_detected2(): No more nearby pedestrians — resuming.", 2);
                    pedestrian_previously_detected = false;
                }
                break;
            }
        }
    }

    bool sign_based_relocalization2(std::shared_ptr<Tracking::KnownStaticObject>& object, double thresh = -1.0) {
        if (thresh < 0) thresh = sign_localization_threshold;
        double error_sq = std::pow(object->x - object->gt_pose[0], 2) + std::pow(object->y - object->gt_pose[1], 2);
        if (error_sq > thresh * thresh) {
            utils.debug("SIGN_RELOC2("+object->name+"): FAILURE: error too large: " + helper::d2str(std::sqrt(error_sq)) + ", thresh: " + helper::d2str(thresh) + ", estimated sign pose: (" + helper::d2str(object->x) + ", " + helper::d2str(object->y) + "), actual: (" + helper::d2str(object->gt_pose[0]) + ", " + helper::d2str(object->gt_pose[1]) + "), sign: " + OBJECT_NAMES[object->type] + ", ID: " + std::to_string(object->id), 2);
            return false;
        }
        double current_yaw = x_current[2];
        double target_yaw = object->gt_pose[2];
        double yaw_error = helper::compare_yaw(target_yaw, current_yaw);
        // check yaw
        if (yaw_error > sign_localization_orientation_threshold * M_PI / 180) {
            utils.debug("SIGN_RELOC2("+object->name+"): FAILURE: yaw error too large: " + helper::d2str(yaw_error * 180/M_PI) + ", thresh: " + helper::d2str(sign_localization_orientation_threshold) + ", sign: " + OBJECT_NAMES[object->type] + ", ID: " + std::to_string(object->id), 2);
            return false;
        }
        double x,y,yaw;
        utils.get_states(x, y, yaw);
        utils.recalibrate_states(object->gt_pose[0] - object->x, object->gt_pose[1] - object->y);
        utils.update_states(x_current);
        utils.debug("SIGN_RELOC2("+object->name+"): SUCCESS: estimated sign pose: (" + helper::d2str(object->x) + ", " + helper::d2str(object->y) + "), actual: (" + helper::d2str(object->gt_pose[0]) + ", " + helper::d2str(object->gt_pose[1]) + "), error: (" + helper::d2str(object->gt_pose[0] - object->x) + ", " + helper::d2str(object->gt_pose[1] - object->y) + "), error norm: " + helper::d2str(std::sqrt(error_sq)) + ", threshold: " + helper::d2str(thresh) + ", old states: (" + helper::d2str(x) + ", " + helper::d2str(y) + "), new states: (" + helper::d2str(x_current[0]) + ", " + helper::d2str(x_current[1]) + "), yaw: " + helper::d2str(x_current[2] * 180 / M_PI) + ", sign: " + OBJECT_NAMES[object->type] + ", ID: " + std::to_string(object->id), 2);
        // stop_for(3.0);
        return true;
    }
    int sign_based_relocalization(const Eigen::Vector2d estimated_sign_pose, const std::vector<std::vector<double>> &EMPIRICAL_POSES, const std::string& sign_type = "", double thresh = -1.0) {
        int min_index = 0;
        double min_error_sq = 1000.0;
        if (thresh < 0) thresh = sign_localization_threshold;
        if (helper::get_min_object_index(estimated_sign_pose, EMPIRICAL_POSES, min_index, min_error_sq, thresh)) {
            double yaw = utils.get_yaw();
            double sign_direction = EMPIRICAL_POSES[min_index][2];
            double yaw_error = helper::compare_yaw(sign_direction, yaw);
            if(yaw_error > sign_localization_orientation_threshold * M_PI / 180) {
                // utils.debug("SIGN_RELOC(" + sign_type + "): FAILURE: yaw error too large: " + helper::d2str(yaw_error) + ", threshold: " + helper::d2str(sign_localization_orientation_threshold), 2);
                return 0;
            }
            double x,y;
            utils.get_states(x, y, yaw);
            utils.recalibrate_states(EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0], EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]);
            utils.update_states(x_current);
            utils.debug("SIGN_RELOC(" + sign_type + "): SUCCESS: estimated sign pose: (" + helper::d2str(estimated_sign_pose[0]) + ", " + helper::d2str(estimated_sign_pose[1]) + "), actual: (" + helper::d2str(EMPIRICAL_POSES[min_index][0]) + ", " + helper::d2str(EMPIRICAL_POSES[min_index][1]) + "), error: (" + helper::d2str(EMPIRICAL_POSES[min_index][0] - estimated_sign_pose[0]) + ", " + helper::d2str(EMPIRICAL_POSES[min_index][1] - estimated_sign_pose[1]) + "), error norm: " + helper::d2str(std::sqrt(min_error_sq)) + ", threshold: " + helper::d2str(sign_localization_threshold) + ", old states: (" + helper::d2str(x) + ", " + helper::d2str(y) + "), new states: (" + helper::d2str(x_current[0]) + ", " + helper::d2str(x_current[1]) + "), yaw: " + helper::d2str(x_current[2] * 180 / M_PI), 2);
            stop_for(3.0);
            PathManager::reset_target_waypoint_index(x_current);
            mpc.reset_solver();
            return 1;
        } else {
            utils.debug("SIGN_RELOC(" + sign_type + "): FAILURE: error too large: " + helper::d2str(std::sqrt(min_error_sq)) + ", threshold: " + helper::d2str(sign_localization_threshold) + ", estimated sign pose: (" + helper::d2str(estimated_sign_pose[0]) + ", " + helper::d2str(estimated_sign_pose[1])  + "), closest: (" + helper::d2str(EMPIRICAL_POSES[min_index][0]) + ", " + helper::d2str(EMPIRICAL_POSES[min_index][1]) + ", " + helper::d2str(EMPIRICAL_POSES[min_index][2]) + ")", 2);
            return 0;
        }
    }
    int intersection_based_relocalization() {
        // stop_for(0.5);
        // check orientation
        double yaw = utils.get_yaw();
        double nearest_direction = helper::nearest_direction(yaw);
        double yaw_error = nearest_direction - yaw;
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        if(std::abs(yaw_error) > intersection_localization_orientation_threshold * M_PI / 180) {
            utils.debug("intersection_based_relocalization(): FAILURE: yaw error too large: " + helper::d2str(yaw_error), 2);
            return 0;
        }

        int nearestDirectionIndex = helper::nearest_direction_index(yaw);
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
        
        int closest_idx = PathManager::closest_waypoint_index;
        int end_idx = std::min(static_cast<int>(closest_idx + lookahead * PathManager::density), static_cast<int>(PathManager::state_refs.rows() - 1));
        int start_idx = std::max(0, closest_idx - static_cast<int>(lookbehind * PathManager::density));
        if (!PathManager::lane_detectable(closest_idx, end_idx)) {
            // utils.debug("LANE_RELOC(): FAILURE: lane not detectable" + helper::d2str(count), 4);
            return 0;
        }
        int num_waypoints = static_cast<int>((lookahead + lookbehind) * PathManager::density);
        double nearest_direction = helper::nearest_direction(utils.get_yaw());
        if (!PathManager::is_straight_line(start_idx, num_waypoints, nearest_direction, 0.1)) {
            // utils.debug("LANE_RELOC(): FAILURE: not a straight line"+ helper::d2str(count), 4);
            return 0;
        }
        utils.update_states(x_current);
        double offset = utils.lane_center_offset;
        if (std::abs(offset) > LANE_OFFSET / 2) {
            // utils.debug("LANE_RELOC(): FAILURE: offset too large: " + helper::d2str(offset), 2);
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
        int nearestDirectionIndex = helper::nearest_direction_index(yaw);
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
                min_error *= 0.75; // TODO: check if this is needed
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
                min_error *= 0.75; // TODO: check if this is needed
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
            int neareastDirection = helper::nearest_direction_index(utils.get_yaw());
            static std::string light_topic_name;
            if (neareastDirection == 0) light_topic_name = "/east_traffic_light";
            else if (neareastDirection == 1) light_topic_name = "/north_traffic_light";
            else if (neareastDirection == 2) light_topic_name = "/west_traffic_light";
            else if (neareastDirection == 3) light_topic_name = "/south_traffic_light";
            auto is_green = ros::topic::waitForMessage<std_msgs::Byte>(light_topic_name, ros::Duration(3));
            while (is_green->data != 1) {
                is_green = ros::topic::waitForMessage<std_msgs::Byte>(light_topic_name, ros::Duration(3));
                utils.publish_cmd_vel(0, 0);
                rate->sleep();
            }
            utils.debug("WAIT_FOR_GREEN(): light turned green, proceeding...", 2);
            return;
        } else {
            utils.debug("WAIT_FOR_GREEN(): red light detected, waiting for " + helper::d2str(stop_duration * 2) + "s or until light turns green", 2);
            auto expiring_time = ros::Time::now() + ros::Duration(3.0);
            int green_count = 0;
            while (true) {
                if (ros::Time::now() > expiring_time) {
                    utils.debug("WAIT_FOR_GREEN(): timer expired, proceeding...", 2);
                    return;
                }
                int sign_index = utils.object_index(OBJECT::GREENLIGHT);
                if (sign_index >= 0) {
                    utils.debug("WAIT_FOR_GREEN(): green light detected, proceeding...", 2);
                    green_count++;
                    if (green_count > 5) return;
                }
                if (sign_index < 0) sign_index = utils.object_index(OBJECT::YELLOWLIGHT);
                if (sign_index >= 0) {
                    utils.debug("WAIT_FOR_GREEN(): yellow light detected, proceeding...", 2);
                    green_count++;
                    if (green_count > 5) return;
                }
                stop_for(T);
            }
            return;
        }
    }
    void publish_waypoints() {
        static Eigen::MatrixXd waypoints = Eigen::MatrixXd::Zero(mpc.N, 3);
        PathManager::get_current_waypoints(waypoints);
        static std_msgs::Float32MultiArray msg;
        msg.data.clear();
        for (int i = 0; i < waypoints.rows(); ++i) {
            msg.data.push_back(waypoints(i, 0)); // x
            msg.data.push_back(waypoints(i, 1)); // y
        }
        utils.waypoints_pub.publish(msg);
        if (utils.tcp_client != nullptr) utils.tcp_client->send_waypoint(msg);
    }
    void orientation_follow(double orientation, double speed = -2) {
        if (speed < -1) speed = NORMAL_SPEED;
        if(pubWaypoints) {
            publish_waypoints();
        }
        utils.update_states(x_current);
        PathManager::find_closest_waypoint(x_current);
        double yaw_error = orientation - utils.get_yaw();
        if(yaw_error > M_PI * 1.5) yaw_error -= 2 * M_PI;
        else if(yaw_error < -M_PI * 1.5) yaw_error += 2 * M_PI;
        double steer = - yaw_error * 180 / M_PI * 1;
        utils.publish_cmd_vel(steer, speed);
    }

    void check_car() {
        int closest_idx = PathManager::closest_waypoint_index;
        double safety_dist = 0.75; // meters
        bool can_overtake = true;
        if (closest_idx < PathManager::overtake_end_index + safety_dist * PathManager::density) return;
        auto cars = Tracking::get_road_cars();
        if (cars.size() == 0) return;
        double min_adj_lane_dist = 1000.;
        double min_adj_lane_lat_dist = 1000.;
        int min_adj_lane_index = 0;
        double min_same_lane_dist = 1000.;
        double min_same_lane_lat_dist = 1000.;
        int min_same_lane_index = 0;
        int start_idx = static_cast<int>(closest_idx); // compute index of midpoint between detected car and ego car
        bool right = false;
        double density = PathManager::density;
        bool on_highway = false;
        double lane_offset = LANE_OFFSET * change_lane_offset_scaler;
        // PathManager::overtake_end_index_scaler = 1.15;
        for (int i = start_idx; i < static_cast<int>(start_idx + 0.5 * PathManager::density); i++) {
            // check if car is on highway
            if (PathManager::attribute_cmp(i, PathManager::ATTRIBUTE::HIGHWAYRIGHT)) { // if on right side of highway, overtake on left
                density *= 1/1.33;
                // PathManager::overtake_end_index_scaler *= 1.5;
                on_highway = true;
                utils.debug("CHECK_CAR(): detected car is on right side of highway, if overtake, on left", 2);
                break;
            }
            else if (PathManager::attribute_cmp(i, PathManager::ATTRIBUTE::HIGHWAYLEFT)) { // if on left side of highway, overtake on right
                right = true; 
                on_highway = true;
                density *= 1/1.33;
                // PathManager::overtake_end_index_scaler *= 1.5;
                break;
            }
        }
        int sign = right ? -1 : 1;
        Eigen::Vector2d start_point_adj = (PathManager::state_refs.block(start_idx, 0, 1, 2).transpose().eval() 
                        + (PathManager::normals.block(start_idx, 0, 1, 2).transpose().eval() 
                        * LANE_OFFSET * sign));
        Eigen::Vector2d start_point_same = PathManager::state_refs.block(start_idx, 0, 1, 2).transpose().eval();
        double car_speed;
        double car_speed_adj;
        for (auto& car : cars) {
            if (car->cumulative_confidence < Tunable::cumulative_confidence_thresholds[static_cast<int>(OBJECT::CAR)]) {
                // std::cout << "CHECK_CAR(): car confidence too low: " + helper::d2str(car->cumulative_confidence) + ", threshold: " + helper::d2str(Tunable::cumulative_confidence_thresholds[static_cast<int>(OBJECT::CAR)]) << std::endl;
                continue;
            }
            if (car->last_detection_time < ros::Time::now() - ros::Duration(Tunable::recency_thresholds[static_cast<int>(OBJECT::CAR)])) continue;

            Eigen::Vector2d car_pose(car->x, car->y);
            double car_yaw = car->yaw;
            double car_dist = (car_pose - x_current.head(2)).norm();
            if (car_dist > min_same_lane_dist && car_dist > min_adj_lane_dist) continue;
            double look_ahead_dist = car_dist  + 0.50;
            int look_ahead_index = look_ahead_dist * PathManager::density + closest_idx;
            double min_dist_sq = 1000.;
            int min_index = 0;
            double min_dist_sq_adj = 1000.; // distance to adjacent lane
            int min_index_adj = 0;
            int lateral_dist_sign = 1;
            int lateral_dist_sign_adj = 1;
            for (int i = closest_idx; i < look_ahead_index; i++) { 
                // iterate over waypoints in front of car, compute distance from car to waypoint, find closest waypoint and distance
                if (i >= PathManager::state_refs.rows()) {
                    break;
                }
                double dist_sq = (car_pose - PathManager::state_refs.block(i, 0, 1, 2).transpose().eval()).squaredNorm();
                if (dist_sq < min_dist_sq) {
                    min_dist_sq = dist_sq;
                    min_index = i;
                }
                Eigen::Vector2d adj_point = (PathManager::state_refs.block(i, 0, 1, 2).transpose().eval() 
                        + (PathManager::normals.block(i, 0, 1, 2).transpose().eval() 
                        * LANE_OFFSET * sign));
                double dist_sq_adj = (car_pose - adj_point).squaredNorm();
                if (dist_sq_adj < min_dist_sq_adj) {
                    min_dist_sq_adj = dist_sq_adj;
                    min_index_adj = i;
                }
            }
            double min_dist = std::sqrt(min_dist_sq);
            double min_dist_adj = std::sqrt(min_dist_sq_adj);
            if (min_dist < min_dist_adj && min_dist < LANE_OFFSET) {
                if (car_dist < min_same_lane_dist) {
                    min_same_lane_dist = car_dist;
                    Eigen::Vector2d same_point = PathManager::state_refs.block(min_index, 0, 1, 2).transpose().eval();
                    Eigen::Vector2d same_point_prev = PathManager::state_refs.block(min_index - 4, 0, 1, 2).transpose().eval();
                    Eigen::Vector2d v1_same = same_point - same_point_prev;
                    Eigen::Vector2d v2_same = car_pose - same_point_prev;
                    double cross_product_same = v1_same(0) * v2_same(1) - v1_same(1) * v2_same(0); // positive if car is on the left of path
                    min_same_lane_lat_dist = min_dist * cross_product_same / std::abs(cross_product_same);
                    min_same_lane_index = min_index;
                    car_speed = car->speed;
                }
            } else if (min_dist_adj < min_dist && min_dist_adj < LANE_OFFSET) {
                if (car_dist < min_adj_lane_dist) {
                    min_adj_lane_dist = car_dist;
                    Eigen::Vector2d adj_point = (PathManager::state_refs.block(min_index, 0, 1, 2).transpose().eval() 
                        + (PathManager::normals.block(min_index, 0, 1, 2).transpose().eval() 
                        * LANE_OFFSET * sign));
                    Eigen::Vector2d adj_point_prev = (PathManager::state_refs.block(min_index - 4, 0, 1, 2).transpose().eval() 
                                + (PathManager::normals.block(min_index - 4, 0, 1, 2).transpose().eval() 
                                * LANE_OFFSET * sign));
                    Eigen::Vector2d v1_adj = adj_point - adj_point_prev;
                    Eigen::Vector2d v2_adj = car_pose - adj_point_prev;
                    double cross_product_adj = v1_adj(0) * v2_adj(1) - v1_adj(1) * v2_adj(0); // positive if car is on the left of adj path
                    min_adj_lane_lat_dist = min_dist_adj * cross_product_adj / std::abs(cross_product_adj);
                    min_adj_lane_index = min_index_adj;
                    car_speed_adj = car->speed;
                }
            }
        }
        double max_car_dist = MAX_CAR_DIST;
        if (on_highway) max_car_dist *= 1.25;
        if (min_same_lane_dist > max_car_dist) {
            // std::cout << "CHECK_CAR(): detected car is too far: " + helper::d2str(min_same_lane_dist) + ", MAX_CAR_DIST: " + helper::d2str(MAX_CAR_DIST) << std::endl;
            return;
        }
        if (std::abs(min_same_lane_lat_dist) > LANE_OFFSET - CAR_WIDTH + SAME_LANE_SAFETY_FACTOR) {
            if (!PathManager::attribute_cmp(min_same_lane_index, PathManager::ATTRIBUTE::INTERSECTION)) {
                // std::cout << "CHECK_CAR(): detected car is not considered on the same lane, min_same_lane_lat_dist: " + helper::d2str(min_same_lane_lat_dist) + ", LANE_OFFSET: " + helper::d2str(LANE_OFFSET) << std::endl;
                return;
            }
            can_overtake = false;
        }
        double min_dist_to_car = Tunable::min_dist_to_car;
        if (on_highway) min_dist_to_car *= 1.3;
        double static_distance = CAR_LENGTH * 1.5 + min_dist_to_car * 2;
        double ego_speed = 0.32;
        if (on_highway) ego_speed *= 1.33;
        car_speed = car_speed;
        double relative_speed = ego_speed - car_speed;
        double total_distance = static_distance * ego_speed / relative_speed;
        int end_idx = static_cast<int>(total_distance * density) + closest_idx;
        can_overtake = can_overtake && (PathManager::attribute_cmp(closest_idx, PathManager::ATTRIBUTE::HIGHWAYLEFT) 
                            || PathManager::attribute_cmp(closest_idx, PathManager::ATTRIBUTE::HIGHWAYRIGHT)
                            || PathManager::attribute_cmp(closest_idx, PathManager::ATTRIBUTE::DOTTED)
                            || PathManager::attribute_cmp(closest_idx, PathManager::ATTRIBUTE::DOTTED_CROSSWALK))
                            && (PathManager::attribute_cmp(end_idx, PathManager::ATTRIBUTE::HIGHWAYLEFT)
                            || PathManager::attribute_cmp(end_idx, PathManager::ATTRIBUTE::HIGHWAYRIGHT)
                            || PathManager::attribute_cmp(end_idx, PathManager::ATTRIBUTE::DOTTED)
                            || PathManager::attribute_cmp(end_idx, PathManager::ATTRIBUTE::DOTTED_CROSSWALK));
        
        if (!can_overtake) {
            utils.debug("CHECK_CAR(): CANT OVERTAKE: detected car is on solid line", 2);
        }
        
        if (relative_speed < 0.15 || total_distance / static_distance > 2.0) {
            utils.debug("CHECK_CAR(): CANT OVERTAKE: detected car is too fast to overtake, relative speed = " + helper::d2str(relative_speed) + ", total distance = " + helper::d2str(total_distance), 2);
            can_overtake = false;
        }
        if (min_adj_lane_dist < total_distance) {
            if (std::abs(min_adj_lane_lat_dist) < LANE_OFFSET - CAR_WIDTH + SAME_LANE_SAFETY_FACTOR) {
                utils.debug("CHECK_CAR(): CANT OVERTAKE: detected car in ADJACENT LANE is too close, dist = " + helper::d2str(min_adj_lane_dist) + ", stopping...", 2);
                can_overtake = false;
            }
        }
        utils.debug("CHECK_CAR(): min_same_lane_dist: " + helper::d2str(min_same_lane_dist) + ", min_adj_lane_dist: " + helper::d2str(min_adj_lane_dist) + ", min_same_lane_lat_dist: " + helper::d2str(min_same_lane_lat_dist) + ", min_adj_lane_lat_dist: " + helper::d2str(min_adj_lane_lat_dist) + ", relative_speed: " + helper::d2str(relative_speed) + ", static_distance: " + helper::d2str(static_distance) + ", total_distance: " + helper::d2str(total_distance) + ", ego_speed: " + helper::d2str(ego_speed) + ", car_speed: " + helper::d2str(car_speed), 2);
        if (can_overtake) {
            stop_for(1.0);
            double start_dist = std::max(min_same_lane_dist - CAR_LENGTH, min_dist_to_car) - Tunable::min_dist_to_car;
            utils.update_states(x_current);
            closest_idx = PathManager::find_closest_waypoint(x_current);
            int start_index = closest_idx + static_cast<int>(start_dist * density);
            // utils.debug("HEREEEEEE!!: start dist: " + helper::d2str(start_dist) + ", min_same_lane_dist: " + helper::d2str(min_same_lane_dist) + ", min_dist_to_car: " + helper::d2str(Tunable::min_dist_to_car) + ", density: " + helper::d2str(density) + ", num index: " + helper::d2str(static_cast<int>(start_dist * density)) + ", closest point: (" + helper::d2str(PathManager::state_refs(closest_idx, 0)) + ", " + helper::d2str(PathManager::state_refs(closest_idx, 1)) + "), min index: " + helper::d2str(min_same_lane_index) + ", min_dist: " + helper::d2str(min_same_lane_dist) + ", min_dist_adj: " + helper::d2str(min_adj_lane_dist), 2);
            if (start_index >= PathManager::state_refs.rows() || PathManager::overtake_end_index >= PathManager::state_refs.rows()) {
                utils.debug("CHECK_CAR(): WARNING: start or end index exceeds state_refs size, stopping...", 2);
                return;
            };
            PathManager::overtake_end_index = start_index + static_cast<int>((total_distance) * density);
            utils.debug("CHECK_CAR(): SAME_LANE: OVERTAKING: start idx: " + helper::d2str(start_index) + ", end idx: " + helper::d2str(PathManager::overtake_end_index) + ", min_dist: " + helper::d2str(min_same_lane_dist) + ", min_dist_adj: " + helper::d2str(min_adj_lane_dist) + "changing lane to the " + std::string(right ? "right" : "left") + " in " + helper::d2str(start_dist) + " meters. start pose: (" + helper::d2str(PathManager::state_refs(start_index, 0)) + "," + helper::d2str(PathManager::state_refs(start_index, 1)) + "), end: (" + helper::d2str(PathManager::state_refs(PathManager::overtake_end_index, 0)) + ", " + helper::d2str(PathManager::state_refs(PathManager::overtake_end_index, 1)) + "), cur: (" + helper::d2str(x_current[0]) + ", " + helper::d2str(x_current[1]) + ", min_dist_to_car: " + helper::d2str(Tunable::min_dist_to_car) + ")", 2);
            int num_extra = PathManager::change_lane(start_index, PathManager::overtake_end_index, right, lane_offset);
            PathManager::overtake_end_index += num_extra;
            return;
        } else {
            if (min_same_lane_dist - CAR_LENGTH < Tunable::min_tailing_dist) {
                mpc.reset_solver();
                utils.debug("CHECK_CAR(): SAME_LANE: cant overtake, dist = " + helper::d2str(min_same_lane_dist) + ", stopping...", 2);
                stop_for(20*T);
                return;
            } else {
                utils.debug("CHECK_CAR(): SAME_LANE: car on oneway pretty far and within safety margin, keep tailing: " + helper::d2str(min_same_lane_dist), 2);
            }
        }
    }
    
    bool goto_command_callback(utils::goto_command::Request &req, utils::goto_command::Response &res) {
        utils.update_states(x_current);
        if (!PathManager::call_go_to_service(x_current[0], x_current[1], x_current[2], req.dest_x, req.dest_y)) {
            res.success = false;
            return false;
        }
        auto state_refs = PathManager::state_refs.transpose();
        auto input_refs = PathManager::input_refs.transpose();
        auto& state_attributes = PathManager::state_attributes;
        auto normals = PathManager::normals.transpose();
        res.state_refs.data = std::vector<float>(state_refs.data(), state_refs.data() + state_refs.size());
        res.input_refs.data = std::vector<float>(input_refs.data(), input_refs.data() + input_refs.size());
        res.wp_attributes.data = std::vector<float>(state_attributes.data(), state_attributes.data() + state_attributes.size());
        res.wp_normals.data = std::vector<float>(normals.data(), normals.data() + normals.size());
        res.success = true;
        destination = PathManager::state_refs.row(PathManager::state_refs.rows()-1).head(2);
        // for (int i = 0; i<PathManager::state_refs.rows(); i++) {
        //     std::cout << i << ") " << PathManager::state_refs(i, 0) << ", " << PathManager::state_refs(i, 1) << ", " << PathManager::state_refs(i, 2) << std::endl;
        // }
        utils.debug("goto_command_callback(): start: " + helper::d2str(x_current(0)) + ", " + helper::d2str(x_current(1)), 2);
        utils.debug("goto_command_callback(): destination: " + helper::d2str(destination(0)) + ", " + helper::d2str(destination(1)), 2);

        PathManager::target_waypoint_index = PathManager::find_closest_waypoint(x_current, 0, PathManager::state_refs.rows()-1); // search from the beginning to the end
        PathManager::find_intersections(utils);
        PathManager::overtake_end_index = 0;
        mpc.reset_solver();
        initialized = true;
        return true;
    }

    bool goto_multiple_command_callback(const std::vector<std::tuple<float, float>> &coords, utils::goto_command::Response &res) {
        utils.update_states(x_current);
        if (!PathManager::call_go_to_multiple_service(x_current[0], x_current[1], x_current[2], coords)) {
            res.success = false;
            return false;
        }
        auto state_refs = PathManager::state_refs.transpose();
        auto input_refs = PathManager::input_refs.transpose();
        auto &state_attributes = PathManager::state_attributes;
        auto normals = PathManager::normals.transpose();
        res.state_refs.data = std::vector<float>(state_refs.data(), state_refs.data() + state_refs.size());
        res.input_refs.data = std::vector<float>(input_refs.data(), input_refs.data() + input_refs.size());
        res.wp_attributes.data = std::vector<float>(state_attributes.data(), state_attributes.data() + state_attributes.size());
        res.wp_normals.data = std::vector<float>(normals.data(), normals.data() + normals.size());
        res.success = true;
        destination = PathManager::state_refs.row(PathManager::state_refs.rows() - 1).head(2);
        // for (int i = 0; i<PathManager::state_refs.rows(); i++) {
        //     std::cout << i << ") " << PathManager::state_refs(i, 0) << ", " << PathManager::state_refs(i, 1) << ", " << PathManager::state_refs(i, 2) << std::endl;
        // }
        utils.debug("goto_command_callback(): start: " + std::to_string(x_current(0)) + ", " + std::to_string(x_current(1)), 2);
        utils.debug("goto_command_callback(): destination: " + std::to_string(destination(0)) + ", " + std::to_string(destination(1)), 2);
        
        PathManager::target_waypoint_index = PathManager::find_closest_waypoint(x_current, 0, PathManager::state_refs.rows() - 1); // search from the beginning to the end
        PathManager::find_intersections(utils);
        PathManager::overtake_end_index = 0;
        mpc.reset_solver();
        initialized = true;
        return true;
    }
    
    bool set_states_callback(utils::set_states::Request &req, utils::set_states::Response &res) {
        if (req.x >= 0 && req.y >= 0) {
            utils.set_states(req.x, req.y);
        } else {
            Sensing::reset_yaw(0);
        }
        res.success = true;
        return true;
    }
};

void StateMachine::update_mpc_states() {
    utils.update_states(x_current);
    double yaw = x_current(2);
    if(PathManager::closest_waypoint_index < PathManager::state_refs.rows() && PathManager::state_refs.rows() > 0) {
        double ref_yaw = PathManager::state_refs(PathManager::closest_waypoint_index, 2);
        yaw = helper::yaw_mod(yaw, ref_yaw);
    }
    x_current(2) = yaw;
}
void StateMachine::solve() {
    // static bool toggle = false;
    // toggle = !toggle;
    // if (!toggle) return;
    // std::cout << "last waypoint index: " << PathManager::last_waypoint_index << ", target waypoint index: " << PathManager::target_waypoint_index << std::endl;
    int success = PathManager::find_next_waypoint(PathManager::target_waypoint_index, x_current);
    // if (PathManager::target_waypoint_index < PathManager::overtake_end_index) {
    //     std::cout << "current state: x: " << x_current(0) << ", y: " << x_current(1) << ", yaw: " << x_current(2) << std::endl;
    //     std::cout << "closest waypoint index: " << PathManager::closest_waypoint_index << ", at x: " << PathManager::state_refs(PathManager::closest_waypoint_index, 0) << ", y: " << PathManager::state_refs(PathManager::closest_waypoint_index, 1) << ", yaw: " << PathManager::state_refs(PathManager::closest_waypoint_index, 2) << std::endl;
    //     std::cout << "target waypoint index: " << PathManager::target_waypoint_index << ", at x: " << PathManager::state_refs(PathManager::target_waypoint_index, 0) << ", y: " << PathManager::state_refs(PathManager::target_waypoint_index, 1) << ", yaw: " << PathManager::state_refs(PathManager::target_waypoint_index, 2) << std::endl;
    //     for (int i = PathManager::target_waypoint_index; i < std::min(PathManager::target_waypoint_index + 6, static_cast<int>(PathManager::state_refs.rows())); i++) {
    //         std::cout << "i: " << i << ", x: " << PathManager::state_refs(i, 0) << ", y: " << PathManager::state_refs(i, 1) << ", yaw: " << PathManager::state_refs(i, 2) << std::endl;
    //     }
    // }
    int idx = PathManager::target_waypoint_index;
    if (idx > PathManager::state_refs.rows()-2) {
        idx = PathManager::state_refs.rows() - 2;
        utils.debug("WARNING: solve(): target waypoint index exceeds state_refs size, using last waypoint...", 3);
    }
    int N = std::min(Eigen::Index(PathManager::N), PathManager::state_refs.rows() - idx);
    if (idx >= 0 && idx <= PathManager::state_refs.rows() - 2 && N > 0) {
        Eigen::Block<Eigen::MatrixXd> state_refs_block = PathManager::state_refs.block(idx, 0, N, 3);
        Eigen::Block<Eigen::MatrixXd> input_refs_block = PathManager::input_refs.block(idx, 0, N, 2);
        int status = mpc.solve(state_refs_block, input_refs_block, x_current);
        
        // x_current(2) = helper::yaw_mod(x_current(2));
        // auto state_refs = utils.waypoints_to_world(utils.lane_waypoints, x_current);
        // state_refs = PathManager::smooth_yaw_angles(state_refs);
        // int status = mpc.solve(state_refs, input_refs_block, x_current);
    } else {
        ROS_WARN("Block indices are out of bounds, skipping solve.");
        ROS_INFO("state_refs rows: %ld, cols: %ld", PathManager::state_refs.rows(), PathManager::state_refs.cols());
        ROS_INFO("input_refs rows: %ld, cols: %ld", PathManager::input_refs.rows(), PathManager::input_refs.cols());
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
    
    // std::cout << "speed: " << speed*100 << ", steer:" << steer << std::endl;

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
        if (Tunable::sign) {
            pedestrian_detected2();
            check_emergency_stop();
        }
        if (state == STATE::MOVING) {
            update_mpc_states();
            solve();
            if(check_intersection()) {
                ;
            }
            if (Tunable::sign) {
                find_sign_for_relocalization();
                find_highway_for_relocalization();
                check_light();
                int park_index = park_sign_detected();
                if(park_index>=0 && park_count < 1) {
                    auto x1 = PARKING_SIGN_POSES1[0][0];
                    auto y1 = PARKING_SIGN_POSES1[0][1];
                    double distance_to_parking_spot = std::sqrt(std::pow((x_current[0] - x1), 2) + std::pow((x_current[1] - y1), 2));
                    double detected_dist = utils.object_distance(park_index);
                    double abs_error = std::abs(detected_dist - distance_to_parking_spot);
                    // double error_threshold = 1.0;
                    double error_threshold = 100.0;
                    if (abs_error < error_threshold ) {
                        utils.debug("parking sign detected, proceeding to parking...", 3);
                        if (sign_relocalize) {
                            auto park_sign_pose = utils.estimate_object_pose2d(x_current[0], x_current[1], x_current[2], utils.object_box(park_index), detected_dist);
                            auto empirical_pose1 = PARKING_SIGN_POSES1[0];
                            double dist_to_empirical_pose1 = std::pow((park_sign_pose[0] - empirical_pose1[0]), 2) + std::pow((park_sign_pose[1] - empirical_pose1[1]), 2);
                            auto empirical_pose2 = PARKING_SIGN_POSES2[0];
                            double dist_to_empirical_pose2 = std::pow((park_sign_pose[0] - empirical_pose2[0]), 2) + std::pow((park_sign_pose[1] - empirical_pose2[1]), 2);
                            auto& empirical_pose = dist_to_empirical_pose1 < dist_to_empirical_pose2 ? PARKING_SIGN_POSES1 : PARKING_SIGN_POSES2;
                            // int success = sign_based_relocalization(park_sign_pose, empirical_pose, "PARKING", 20.0); // relocalize to parking sign
                            int success = sign_based_relocalization(park_sign_pose, empirical_pose, "PARKING"); // relocalize to parking sign
                        }
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
            if (error_sq < TOLERANCE_SQUARED && PathManager::target_waypoint_index >= PathManager::state_refs.rows() * 0.9) {
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
                double orientation = helper::nearest_direction(utils.get_yaw());
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
                utils.debug("PARKING(): target spot: " + helper::d2str(target_spot), 2);
                for(int i = 0; i<GroundTruth::PARKING_SPOTS.size(); i++) {
                    std::cout << "parking spot " << i << ": " << GroundTruth::PARKING_SPOTS[i][0] << ", " << GroundTruth::PARKING_SPOTS[i][1] << std::endl;
                }
                while(1) {
                    pedestrian_detected2();
                    check_emergency_stop();
                    auto cars = Tracking::get_road_cars();
                    bool changed = false;
                    bool car_in_spot = false;
                    while(1) {
                        for (auto car: cars) {
                            Eigen::Vector2d world_pose = Eigen::Vector2d(car->x, car->y);
                            const Eigen::Vector2d& spot = GroundTruth::PARKING_SPOTS[target_spot];
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
                        ROS_INFO("car in spot, changing to target spot %d at (%.3f, %.3f), right: %s", target_spot, GroundTruth::PARKING_SPOTS[target_spot][0], GroundTruth::PARKING_SPOTS[target_spot][1], right_park ? "true" : "false");
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
                double orientation = helper::nearest_direction(utils.get_yaw());
                double x, y, yaw;
                utils.get_states(x, y, yaw);
                double initial_y_error = y - (GroundTruth::PARKING_SPOTS[target_spot][1] + PARKING_SPOT_WIDTH * (right_park ? 1 : -1));
                double initial_yaw_error = orientation - yaw;
                initial_yaw_error = helper::yaw_mod(initial_yaw_error); // normalize to [-pi, pi]
                // ROS_INFO("initial y error: %.3f, initial yaw error: %.3f", initial_y_error, initial_yaw_error);
                utils.debug("orientation: " + helper::d2str(orientation) + ", yaw: " + helper::d2str(yaw), 4);
                // exit(0);
                parking_maneuver_hardcode(right_park, false, 1/T_park, initial_y_error, initial_yaw_error);
            }
            double x, y, yaw;
            utils.get_states(x, y, yaw);
            double x_error = x - GroundTruth::PARKING_SPOTS[target_spot][0];
            if (std::abs(x_error) > 0.15) {
                double orientation = helper::nearest_direction(utils.get_yaw());
                ROS_INFO("parked but x offset too large: %.3f, adjusting... orientation: %.3f", x_error, orientation);
                double x0, y0, yaw0;
                utils.get_states(x0, y0, yaw0);
                while(1) {
                    x_error = x - GroundTruth::PARKING_SPOTS[target_spot][0];
                    pedestrian_detected2();
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
            PathManager::target_waypoint_index = PathManager::find_closest_waypoint(x_current);
            std::cout << "exiting_park(): target waypoint index: " << PathManager::target_waypoint_index << ", at (" << PathManager::state_refs(PathManager::target_waypoint_index, 0) << ", " << PathManager::state_refs(PathManager::target_waypoint_index, 1) << ")" << std::endl;
            std::cout << "exiting_park(): closest waypoint index: " << PathManager::closest_waypoint_index << ", at (" << PathManager::state_refs(PathManager::closest_waypoint_index, 0) << ", " << PathManager::state_refs(PathManager::closest_waypoint_index, 1) << ")" << std::endl;
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
    if (Sensing::serial && Sensing::serial->is_open()) {
        Sensing::serial->close();
    }
    Sensing::serial.reset();
    ros::shutdown();
    exit(signum);
}

int main(int argc, char **argv) {
    std::cout.precision(3);
    ros::init(argc, argv, "mpc_node", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
    ros::NodeHandle nh;

    Database db;
    VehicleConstants::init_params(db);

    if (!Tunable::loadFromParams(nh)) {
        std::cout << "FATAL ERROR: Failed to load tunable parameters" << std::endl;
        exit(1);
    }
    GroundTruth::initialize_ground_truth();
    Tracking::initialize_tracking();
    Sensing::initialize_sensing(nh);
    StateMachine sm(nh, db);

    globalStateMachinePtr = &sm;
    signal(SIGINT, signalHandler);
    
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

    ros::waitForShutdown();
    if (callback_thread.joinable()) {
        callback_thread.join();
    }
    return 0;
}
