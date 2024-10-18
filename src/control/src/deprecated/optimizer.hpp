#ifndef OPTIMIZER_HPP
#define OPTIMIZER_HPP

#include <Eigen/Dense>
#include <chrono>
#include <iostream>
#include <stdio.h>
#include <stdlib.h>
#include <fstream>
#include <unistd.h>
#include <limits.h>
#include <cmath>
#include "acados/utils/math.h"
#include "acados_c/ocp_nlp_interface.h"
#include "acados_sim_solver_mobile_robot.h"
#include "acados_solver_mobile_robot.h"
#include "acados_sim_solver_mobile_robot_25.h"
#include "acados_solver_mobile_robot_25.h"
#include "acados_sim_solver_mobile_robot_18.h"
#include "acados_solver_mobile_robot_18.h"
#include "acados_solver_park.h"
#include "acados_sim_solver_park.h"
#include "constants.h"

class Optimizer {
public:
    Optimizer(double T, int N, double v_ref, double x_init, double y_init, double yaw_init);
    Optimizer(double T, int N, double v_ref): Optimizer(T, N, v_ref, 0.83, 0.33, 1.65756) {}
    Optimizer(): Optimizer(0.125, 40, 1.0, 0.83, 0.33, 1.65756) {}
    ~Optimizer() {
        free(acados_ocp_capsule);
        free(sim_capsule);
        free(mobile_robot_sim_config);
        free(mobile_robot_sim_dims);
        free(mobile_robot_sim_in);
        free(mobile_robot_sim_out);
        free(nlp_config);
        free(nlp_dims);
        free(nlp_in);
        free(nlp_out);

        free(acados_ocp_capsule_park);
        free(sim_capsule_park);
        free(park_sim_config);
        free(park_sim_dims);
        free(park_sim_in);
        free(park_sim_out);
        free(nlp_config_park);
        free(nlp_dims_park);
        free(nlp_in_park);
        free(nlp_out_park);
    }
    int run(); 
    int update_and_solve(Eigen::Vector3d &i_current_state, bool safety_check = true, int mode = -1);
    void integrate_next_states();
    int find_next_waypoint(int &output_target, Eigen::Vector3d &i_current_state, bool safety_check = true, int min_index = -1, int max_index = -1);
    int find_closest_waypoint(int min_index = -1, int max_index = -1);
    int update_current_states(double x, double y, double yaw, Eigen::Vector3d& state, bool safety_check = true);
    int update_current_states(double x, double y, double yaw, bool safety_check = true) {
        return update_current_states(x, y, yaw, x_current, safety_check);
    }
    int initialize_current_states(double x, double y, double yaw) {
        return update_current_states(x, y, yaw, x_current, false);
    }
    // void update_current_states(double* state);
    Eigen::VectorXd computeStats(int hsy);
    static std::string getSourceDirectory();
    template <typename EigenType>
    void saveToFile(const EigenType &data, const std::string &filename);
    Eigen::MatrixXd loadTxt(const std::string &filename);

// private:
    int status; // acados operation state
    int idxbx0[3];
    double min_time;
    double elapsed_time;
    double x_state[5];
    // double x_current[3];
    Eigen::Vector3d x_current;
    Eigen::Vector3d x_real;
    double u_current[2];
    int N, nx, nu, iter = 0;
    int N_park, nx_park, nu_park;
    double T_park;
    int target_waypoint_index=0, last_waypoint_index=0, closest_waypoint_index=0, num_waypoints=0;
    int v_ref_int;
    bool use25 = false;
    bool use18 = false;
    double region_of_acceptance, region_of_acceptance_cw, region_of_acceptance_hw, v_ref, t0, T, density, rdb_circumference = 3.95;
    bool debug = true;
    Eigen::Vector3d current_state;
    Eigen::MatrixXd state_refs, input_refs, normals, left_turn_states, right_turn_states, straight_states;
    Eigen::VectorXd state_attributes;
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
    Eigen::MatrixXd *state_refs_ptr;
    Eigen::VectorXd distances;

    mobile_robot_solver_capsule *acados_ocp_capsule;
    mobile_robot_sim_solver_capsule *sim_capsule;
    mobile_robot_25_solver_capsule *acados_ocp_capsule_25;
    mobile_robot_25_sim_solver_capsule *sim_capsule_25;
    mobile_robot_18_solver_capsule *acados_ocp_capsule_18;
    mobile_robot_18_sim_solver_capsule *sim_capsule_18;
    sim_config *mobile_robot_sim_config;
    void *mobile_robot_sim_dims;
    sim_in *mobile_robot_sim_in;
    sim_out *mobile_robot_sim_out;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;

    park_solver_capsule *acados_ocp_capsule_park;
    park_sim_solver_capsule *sim_capsule_park;
    sim_config *park_sim_config;
    void *park_sim_dims;
    sim_in *park_sim_in;
    sim_out *park_sim_out;
    ocp_nlp_config *nlp_config_park;
    ocp_nlp_dims *nlp_dims_park;
    ocp_nlp_in *nlp_in_park;
    ocp_nlp_out *nlp_out_park;

    Eigen::MatrixXd simX, simU, time_record;
    Eigen::VectorXd x_errors, y_errors, yaw_errors;

    Eigen::Matrix2d rotation_matrix;
    Eigen::Vector2d rotated_xy;
    void transform_point(Eigen::Vector3d& pt, Eigen::Vector3d& pt_transformed,
                         const Eigen::Vector3d& frame1, 
                         const Eigen::Vector3d& frame2) {
        // Unpack the frames and the point
        const double &x1 = frame1[0], &y1 = frame1[1], &theta1 = frame1[2];
        const double &x2 = frame2[0], &y2 = frame2[1], &theta2 = frame2[2];
        double &x = pt[0], &y = pt[1], &psi = pt[2];
        double &x_transformed = pt_transformed[0], &y_transformed = pt_transformed[1], &psi_transformed = pt_transformed[2];

        // Step 1: Translate to the origin of frame1
        x_transformed = x - x1;
        y_transformed = y - y1;

        // Step 2: Rotate to align frame1 with frame2
        double rotation_angle = theta2 - theta1;
        
        rotation_matrix << cos(rotation_angle), -sin(rotation_angle),
                           sin(rotation_angle),  cos(rotation_angle);
        rotated_xy = rotation_matrix * pt_transformed.head(2);

        // Update psi (yaw) and normalize
        psi_transformed = std::fmod(psi + rotation_angle, 2 * M_PI);

        // Step 3: Translate to the origin of frame2
        x_transformed = rotated_xy[0] + x2;
        y_transformed = rotated_xy[1] + y2;
    }
    void set_up_park(Eigen::VectorXd& xs) {
        int reset_status = park_acados_reset(acados_ocp_capsule_park, 1);
        if( reset_status ) {
            printf("WARNING: park_acados_reset() returned status %d.\n", reset_status);
        }
        ocp_nlp_cost_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, N_park, "yref", xs.data());
        std::cout << "N_park: " << N_park << std::endl;
        for (int j = 0; j < N_park; ++j) {
            ocp_nlp_cost_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, j, "yref", xs.data());
        }
        std::cout << "frame1: " << frame1 << std::endl;
        std::cout << "x_current: " << x_current << std::endl;
        frame1.head(2) = x_current.head(2);
        frame1[2] = NearestDirection(x_current[2]);
        std::cout << "yaw: " << x_current[2] << ", nearest: " << frame1[2] << std::endl;
        frame2 << 0.0, 0.0, M_PI;
        frame2 << 0.0, 0.0, 0.0;
        std::cout << "frame1: " << frame1[0] << ", " << frame1[1] << ", " << frame1[2] << std::endl;
        std::cout << "frame2: " << frame2[0] << ", " << frame2[1] << ", " << frame2[2] << std::endl;
    }
    Eigen::Vector3d frame2;
    Eigen::Vector3d frame1;
    Eigen::Vector3d x_current_transformed;
    
    int update_and_solve_park(Eigen::VectorXd& xs, double thresh_sq = 0.0025) {
        auto t_start = std::chrono::high_resolution_clock::now();
        
        transform_point(x_current, x_current_transformed, frame1, frame2);
        double yaw_frame2 = frame2[2];
        while (x_current_transformed[2] - yaw_frame2 > M_PI) {
            x_current_transformed[2] -= 2 * M_PI;
        }
        while (x_current_transformed[2] - yaw_frame2 < -M_PI) {
            x_current_transformed[2] += 2 * M_PI;
        }
        double error_sq = (x_current_transformed - xs.head(3)).squaredNorm();
        static int count = 0;
        // std::cout << count << ")error: "<< sqrt(error_sq) <<", x_cur: " << x_current[0] << ", " << x_current[1] << ", " << x_current[2] << ", x_cur_trans: " << x_current_transformed[0] << ", " << x_current_transformed[1] << ", " << x_current_transformed[2] << ", xs: " << xs[0] << ", " << xs[1] << ", " << xs[2] << ", u: " << u_current[0] << ", " << u_current[1] << std::endl;
        count ++;
        if (error_sq < thresh_sq) {
            count = 0;
            printf("threshold reached, error: %4f\n", sqrt(error_sq));
            return 2;
        }
        ocp_nlp_constraints_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, 0, "lbx", x_current_transformed.data());
        ocp_nlp_constraints_model_set(nlp_config_park, nlp_dims_park, nlp_in_park, 0, "ubx", x_current_transformed.data());

        // Solve the optimization problem
        status = park_acados_solve(acados_ocp_capsule_park);
        if (status != 0) {
            std::cout << "ERROR!!! acados acados_ocp_solver returned status " << status << ". Exiting." << std::endl;
            return 1; 
        }

        // Get the optimal control for the next step
        ocp_nlp_out_get(nlp_config_park, nlp_dims_park, nlp_out_park, 0, "u", &u_current);
        if (debug) {
            auto t_end = std::chrono::high_resolution_clock::now();
            double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
            // std::cout << "elapsed time: " << elapsed_time_ms << " ms" << std::endl;
            time_record(iter) = elapsed_time_ms;
            for(int i=0; i<nu; i++) {
                simU(iter, i) = u_current[i];
            }
            for(int ii=0; ii<nx; ii++) {
                simX(iter, ii) = x_current[ii];
            }
            // std::cout << iter<< ") x_cur: " << x_current[0] << ", " << x_current[1] << ", " << x_current[2] << ", ref: " << state_refs(idx, 0) << ", " << state_refs(idx, 1) << ", " << state_refs(idx, 2) << ", u: " << u_current[0] << ", " << u_current[1] << ", error: " << error << std::endl;
            iter++;
        }
        return 0;
    }

    double NearestDirection(double yaw) {
        while(yaw > 2 * M_PI) {
            yaw -= 2 * M_PI;
        }
        while(yaw < 0) {
            yaw += 2 * M_PI;
        }

        static const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};

        double minDifference = std::abs(yaw - directions[0]);
        double nearestDirection = directions[0];

        for (int i = 1; i < 5; ++i) {
            double difference = std::abs(yaw - directions[i]);
            if (difference < minDifference) {
                minDifference = difference;
                nearestDirection = directions[i];
            }
        }
        while (nearestDirection - yaw > M_PI) {
            nearestDirection -= 2 * M_PI;
        }
        while (nearestDirection - yaw < -M_PI) {
            nearestDirection += 2 * M_PI;
        }
        return nearestDirection;
    }
    int NearestDirectionIndex(double yaw) {
        while(yaw > 2 * M_PI) {
            yaw -= 2 * M_PI;
        }
        while(yaw < 0) {
            yaw += 2 * M_PI;
        }

        static const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};

        double minDifference = std::abs(yaw - directions[0]);
        double nearestDirection = directions[0];

        int closest_index = 0;
        for (int i = 1; i < 5; ++i) {
            double difference = std::abs(yaw - directions[i]);
            if (difference < minDifference) {
                minDifference = difference;
                nearestDirection = directions[i];
                closest_index = i;
            }
        }
        if (closest_index == 4) {
            closest_index = 0;
        }
        return closest_index;
    }

    int park() {
        std::cout.precision(3);
        //print T, N, nx, nu
        std::cout << "T: " << T_park << ", N: " << N_park << ", nx: " << nx_park << ", nu: " << nu_park << std::endl;
        Eigen::VectorXd xs(5);
        // xs << -1.31, 0, M_PI, 0, 0;
        xs << 0.63, 0.32, M_PI, 0, 0;
        xs << -0.63, -0.32, 0.0, 0, 0;
        set_up_park(xs);
        int i = 0;
        while(1) {
            status = update_and_solve_park(xs);
            if (status == 2 || i>100) {
                break;
            }
            x_current[0] += 0.05 * u_current[0] * cos(x_current[2]); // v * cos(psi) * dt
            x_current[1] += 0.05 * u_current[0] * sin(x_current[2]); // v * sin(psi) * dt
            x_current[2] += 0.05 * u_current[0]/0.27 * tan(u_current[1]); // v * tan(delta) * dt / L
            i++;
        }
        std::cout << "done1" << std::endl;
        xs << 0, 0, M_PI, 0, 0;
        xs << 0.63, 0.32, 0, 0, 0;
        set_up_park(xs);
        i = 0;
        while(1) {
            status = update_and_solve_park(xs);
            if (status == 2 || i>100) {
                break;
            }
            x_current[0] += 0.05 * u_current[0] * cos(x_current[2]); 
            x_current[1] += 0.05 * u_current[0] * sin(x_current[2]); 
            x_current[2] += 0.05 * u_current[0]/0.27 * tan(u_current[1]); 
            i++;
        }
        std::cout << "done2" << std::endl;
        return 0;
    }

    void change_lane(int start_index, int end_index, bool shift_right = false, double shift_distance = 0.36-0.1) {
        if (shift_right) shift_distance *= -1;
        // state_refs.block(start_index, 0, end_index-start_index, 2) += normals.block(start_index, 0, end_index-start_index, 2) * shift_distance;

        // Total number of points
        int total_points = end_index - start_index;
        // int ramp_length = static_cast<int>(density * VehicleConstants::CAR_LENGTH / 2);
        int ramp_length = static_cast<int>(density * 0.125);
        std::cout << "ramp_length (#wpts): " << ramp_length << std::endl;

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

        // // Calculate the vectors for the gaps
        // Eigen::Vector2d start_gap_vector = state_refs.row(start_index).head(2) - state_refs.row(start_index - 1).head(2);
        // Eigen::Vector2d end_gap_vector = state_refs.row(end_index + 1).head(2) - state_refs.row(end_index).head(2);
        // std::cout << "start_gap_vector: " << start_gap_vector.transpose() << std::endl;
        // std::cout << "start: " << state_refs.row(start_index) << ", start - 1: " << state_refs.row(start_index - 1) << std::endl;
        // double start_gap_yaw = std::atan2(start_gap_vector.y(), start_gap_vector.x());
        // double end_gap_yaw = std::atan2(end_gap_vector.y(), end_gap_vector.x());

        // std::cout << "start_gap_yaw: " << start_gap_yaw << std::endl;

        // // Calculate the number of points to interpolate based on the actual length of the gap
        // int num_start_points = static_cast<int>(density * start_gap_vector.norm() / 2);
        // int num_end_points = static_cast<int>(density * end_gap_vector.norm() / 2);

        // std::cout << "num_start_points: " << num_start_points << std::endl;
        // std::cout << "num_end_points: " << num_end_points << std::endl;

        // // Create matrices to hold the new interpolated waypoints for the start and end gaps
        // Eigen::MatrixXd start_gap_filler(num_start_points, 3);
        // Eigen::MatrixXd end_gap_filler(num_end_points, 3);

        // // Interpolate for the start gap
        // for (int i = 0; i < num_start_points; ++i) {
        //     double t = static_cast<double>(i + 1) / (num_start_points + 1);
        //     Eigen::Vector2d interpolated_position = state_refs.row(start_index - 1).head(2) * (1 - t) + 
        //                                             state_refs.row(start_index).head(2) * t;
        //     start_gap_filler.row(i).head(2) = interpolated_position;
        //     start_gap_filler(i, 2) = start_gap_yaw;
        // }
        // std::cout << "start_gap_filler: " << start_gap_filler << std::endl;
        // // Interpolate for the end gap
        // for (int i = 0; i < num_end_points; ++i) {
        //     double t = static_cast<double>(i + 1) / (num_end_points + 1);
        //     Eigen::Vector2d interpolated_position = state_refs.row(end_index).head(2) * (1 - t) + 
        //                                             state_refs.row(end_index + 1).head(2) * t;
        //     end_gap_filler.row(i).head(2) = interpolated_position;
        //     end_gap_filler(i, 2) = end_gap_yaw;
        // }
        // int original_size = state_refs.rows();
        // int new_size = original_size + start_gap_filler.rows() + end_gap_filler.rows();
        // std::cout << "original_size: " << original_size << std::endl;
        // std::cout << "new_size: " << new_size << std::endl;
        // state_refs.conservativeResize(new_size, Eigen::NoChange); 

        // std::cout << "state_refs size: " << state_refs.rows() << ", " << state_refs.cols() << std::endl;
        // // Shift the end part of the matrix to make space for the 'end_gap_filler'
        // state_refs.block(end_index + 1 + end_gap_filler.rows(), 0, original_size - end_index - 1, 3) =
        //     state_refs.block(end_index + 1, 0, original_size - end_index - 1, 3);

        // std::cout << "state_refs size2: " << state_refs.rows() << ", " << state_refs.cols() << std::endl;
        // // Insert 'end_gap_filler' into 'state_refs'
        // state_refs.block(end_index + 1, 0, end_gap_filler.rows(), 3) = end_gap_filler;

        // std::cout << "state_refs size3: " << state_refs.rows() << ", " << state_refs.cols() << std::endl;
        // // Shift the middle part of the matrix to make space for the 'start_gap_filler'
        // state_refs.block(start_index + start_gap_filler.rows(), 0, new_size - start_index - start_gap_filler.rows(), 3) =
        //     state_refs.block(start_index, 0, new_size - start_index - start_gap_filler.rows(), 3);

        // std::cout << "state_refs size4: " << state_refs.rows() << ", " << state_refs.cols() << std::endl;
        // // Insert 'start_gap_filler' into 'state_refs'
        // state_refs.block(start_index, 0, start_gap_filler.rows(), 3) = start_gap_filler;
        // std::cout << "state_refs size5: " << state_refs.rows() << ", " << state_refs.cols() << std::endl;
        // for (int i = start_index -10; i < start_index + 30; ++i) {
        //     std::cout << "state_refs(" << i << "): " << state_refs(i + start_index -10, 0) << ", " << state_refs(i + start_index -10, 1) << ", " << state_refs(i + start_index -10, 2) << std::endl;
        // }
    }
    int get_current_attribute() {
        return state_attributes(target_waypoint_index);
    }
    void get_current_waypoints(Eigen::MatrixXd& output) {
        output = state_refs.block(target_waypoint_index, 0, N, 3);
    }
    int reset_solver() {
        int reset_status;
        if(use25) {
            reset_status = mobile_robot_25_acados_reset(acados_ocp_capsule_25, 1);
        } else if(use18) {
            reset_status = mobile_robot_18_acados_reset(acados_ocp_capsule_18, 1);
        } else {
            reset_status = mobile_robot_acados_reset(acados_ocp_capsule, 1);
        }
        return reset_status;
    }
};

#endif // OPTIMIZER_HPP