#include "optimizer.hpp"

Optimizer::Optimizer(double T, int N, double v_ref, double x_init, double y_init, double yaw_init):
    T(T), N(N), v_ref(v_ref), density(1/T/v_ref), region_of_acceptance(0.03076923*3 * (0.125*1.3) / density), 
    region_of_acceptance_cw(region_of_acceptance * 1.0/1.5), region_of_acceptance_hw(region_of_acceptance * 1.5), t0(0.0), closest_waypoint_index(0)
 {
    std::cout << "Optimizer Constructor" << std::endl;
    v_ref_int = static_cast<int>(v_ref * 100); // convert to cm/s
    // Initialize member variables
    min_time = 1e12;
    status = 0; // Assuming 0 is a default 'no error' state
    
    acados_ocp_capsule_park = park_acados_create_capsule();
    int status2 = park_acados_create(acados_ocp_capsule_park);
    if (status2) {
        printf("park_acados_create() returned status %d. Exiting.\n", status2);
        exit(1);
    }
    sim_capsule_park = park_acados_sim_solver_create_capsule();
    status2 = park_acados_sim_create(sim_capsule_park);
    park_sim_config = park_acados_get_sim_config(sim_capsule_park);
    park_sim_dims = park_acados_get_sim_dims(sim_capsule_park);
    park_sim_in = park_acados_get_sim_in(sim_capsule_park);
    park_sim_out = park_acados_get_sim_out(sim_capsule_park);
    if (status2) {
        printf("acados_create() simulator returned status %d. Exiting.\n", status2);
        exit(1);
    }
    nlp_config_park = park_acados_get_nlp_config(acados_ocp_capsule_park);
    nlp_dims_park = park_acados_get_nlp_dims(acados_ocp_capsule_park);
    nlp_in_park = park_acados_get_nlp_in(acados_ocp_capsule_park);
    nlp_out_park = park_acados_get_nlp_out(acados_ocp_capsule_park);

    if (v_ref_int >= 30) {
        std::cout << "reference speed is 35. cm/s" << std::endl;
        // Create a capsule according to the pre-defined model
        acados_ocp_capsule = mobile_robot_acados_create_capsule();
        // Initialize the optimizer
        std::cout << "hi11" <<std::endl;
        status = mobile_robot_acados_create(acados_ocp_capsule);
        std::cout << "hi22" <<std::endl;
        if (status) {
            printf("mobile_robot_acados_create() returned status %d. Exiting.\n", status);
            exit(1);
        }
        // Create and initialize simulator capsule
        std::cout << "hi" <<std::endl;
        sim_capsule = mobile_robot_acados_sim_solver_create_capsule();
        std::cout << "hi0" <<std::endl;
        status = mobile_robot_acados_sim_create(sim_capsule);
        std::cout << "hi" <<std::endl;

        mobile_robot_sim_config = mobile_robot_acados_get_sim_config(sim_capsule);
        mobile_robot_sim_dims = mobile_robot_acados_get_sim_dims(sim_capsule);
        mobile_robot_sim_in = mobile_robot_acados_get_sim_in(sim_capsule);
        mobile_robot_sim_out = mobile_robot_acados_get_sim_out(sim_capsule);

        std::cout << "hi2" <<std::endl;
        if (status) {
            printf("acados_create() simulator returned status %d. Exiting.\n", status);
            exit(1);
        }

        // Initialize some important structure of ocp
        nlp_config = mobile_robot_acados_get_nlp_config(acados_ocp_capsule);
        nlp_dims = mobile_robot_acados_get_nlp_dims(acados_ocp_capsule);
        std::cout << "hi3" <<std::endl;
        nlp_in = mobile_robot_acados_get_nlp_in(acados_ocp_capsule);
        nlp_out = mobile_robot_acados_get_nlp_out(acados_ocp_capsule);
    } else if(v_ref_int == 25) {
        std::cout << "reference speed is 25 cm/s" << std::endl;
        use25 = true;
        acados_ocp_capsule_25 = mobile_robot_25_acados_create_capsule();
        status = mobile_robot_25_acados_create(acados_ocp_capsule_25);
        if (status) {
            printf("mobile_robot_25_acados_create() returned status %d. Exiting.\n", status);
            exit(1);
        }
        sim_capsule_25 = mobile_robot_25_acados_sim_solver_create_capsule();
        status = mobile_robot_25_acados_sim_create(sim_capsule_25);
        mobile_robot_sim_config = mobile_robot_25_acados_get_sim_config(sim_capsule_25);
        mobile_robot_sim_dims = mobile_robot_25_acados_get_sim_dims(sim_capsule_25);
        mobile_robot_sim_in = mobile_robot_25_acados_get_sim_in(sim_capsule_25);
        mobile_robot_sim_out = mobile_robot_25_acados_get_sim_out(sim_capsule_25);
        if (status) {
            printf("acados_create() simulator returned status %d. Exiting.\n", status);
            exit(1);
        }
        nlp_config = mobile_robot_25_acados_get_nlp_config(acados_ocp_capsule_25);
        nlp_dims = mobile_robot_25_acados_get_nlp_dims(acados_ocp_capsule_25);
        nlp_in = mobile_robot_25_acados_get_nlp_in(acados_ocp_capsule_25);
        nlp_out = mobile_robot_25_acados_get_nlp_out(acados_ocp_capsule_25);
    } else if(v_ref_int == 18) {
        std::cout << "reference speed is 18 cm/s" << std::endl;
        use18 = true;
        use25 = false;
        acados_ocp_capsule_18 = mobile_robot_18_acados_create_capsule();
        status = mobile_robot_18_acados_create(acados_ocp_capsule_18);
        if (status) {
            printf("mobile_robot_18_acados_create() returned status %d. Exiting.\n", status);
            exit(1);
        }
        sim_capsule_18 = mobile_robot_18_acados_sim_solver_create_capsule();
        status = mobile_robot_18_acados_sim_create(sim_capsule_18);
        mobile_robot_sim_config = mobile_robot_18_acados_get_sim_config(sim_capsule_18);
        mobile_robot_sim_dims = mobile_robot_18_acados_get_sim_dims(sim_capsule_18);
        mobile_robot_sim_in = mobile_robot_18_acados_get_sim_in(sim_capsule_18);
        mobile_robot_sim_out = mobile_robot_18_acados_get_sim_out(sim_capsule_18);
        if (status) {
            printf("acados_create() simulator returned status %d. Exiting.\n", status);
            exit(1);
        }
        nlp_config = mobile_robot_18_acados_get_nlp_config(acados_ocp_capsule_18);
        nlp_dims = mobile_robot_18_acados_get_nlp_dims(acados_ocp_capsule_18);
        nlp_in = mobile_robot_18_acados_get_nlp_in(acados_ocp_capsule_18);
        nlp_out = mobile_robot_18_acados_get_nlp_out(acados_ocp_capsule_18);
    } else {
        std::cerr << "Invalid reference speed, please use 18, 25 or 50" << std::endl;
        exit(1);
    }

    // Setting problem dimensions
    N = nlp_dims->N;
    nx = *nlp_dims->nx;
    nu = *nlp_dims->nu;

    N_park = nlp_dims_park->N;
    nx_park = *nlp_dims_park->nx;
    nu_park = *nlp_dims_park->nu;

    // Initialize target, current state and state variables
    x_current[0] = x_init;
    x_current[1] = y_init;
    x_current[2] = yaw_init;

    x_state[0] = 0.0;
    x_state[1] = 0.0;
    x_state[2] = 0.0;
    x_state[3] = 0.0;
    x_state[4] = 0.0;

    // region_of_acceptance = 0.03076923*3.0 * (0.125*1.3) / density;
    // get relative path
    std::string dir = getSourceDirectory();
    dir.replace(dir.rfind("src"), 3, "scripts");
    std::string v_ref_int_str = std::to_string(v_ref_int);
    std::string path_name = "_speedrun";
    // path_name = "";
    path_name = "_path1";
    state_refs = loadTxt(dir + "/paths/state_refs" + path_name +v_ref_int_str+ ".txt");
    input_refs = loadTxt(dir + "/paths/input_refs" + path_name +v_ref_int_str+ ".txt");
    state_attributes = loadTxt(dir + "/paths/wp_attributes" + path_name +v_ref_int_str+ ".txt");
    // left_turn_states = loadTxt(dir + "/paths/left_turn_states" + path_name +v_ref_int_str+ ".txt");
    // right_turn_states = loadTxt(dir + "/paths/right_turn_states" + path_name +v_ref_int_str+ ".txt");
    // straight_states = loadTxt(dir + "/paths/straight_states" + path_name +v_ref_int_str+ ".txt");
    // state_refs = state_refs.block(3026, 0, 98, 3);
    std::vector<int> indices;
    for(int i=0; i<state_attributes.rows(); i++) {
        if(state_attributes(i) == 7) {
            indices.push_back(i);
        }
    }
    state_refs_ptr = &state_refs;
    // normals = loadTxt("/home/simonli/bfmc_pkgs/mpc/scripts/paths/wp_normals2.txt");
    normals = loadTxt(dir + "/paths/wp_normals"+ path_name +v_ref_int_str+ ".txt");
    num_waypoints = state_refs.rows();

    target_waypoint_index = 0;
    last_waypoint_index = target_waypoint_index;
    // region_of_acceptance = 0.03076923*2*1.5;

    int len = static_cast<int>(num_waypoints * 2);
    simX = Eigen::MatrixXd::Zero(len, nx); 
    simU = Eigen::MatrixXd::Zero(len, nu);
    x_errors = Eigen::VectorXd::Zero(len, 1);
    y_errors = Eigen::VectorXd::Zero(len, 1);
    yaw_errors = Eigen::VectorXd::Zero(len, 1);
    time_record = Eigen::MatrixXd::Zero(len, nu);
    std::cout << "v ref: " << v_ref << ", int:" << v_ref_int << std::endl;
    if(v_ref > 0.3) v_ref = 0.35;
}

int Optimizer::run() {
    std::cout.precision(3);
    int hsy = 0;
    Eigen::VectorXd x_final = state_refs.row(state_refs.rows() - 1);
    while(1) {
        double error_norm = (x_final - x_current).norm();
        if(target_waypoint_index > num_waypoints || error_norm < 0.1) {
            break;
        }
        std::cout << "target_waypoint_index: " << target_waypoint_index << ", hsy: " << hsy << ", norm: " << error_norm << std::endl;
        // x_errors(hsy) = state_refs(target_waypoint_index, 0) - x_current[0];
        // y_errors(hsy) = state_refs(target_waypoint_index, 1) - x_current[1];
        // yaw_errors(hsy) = state_refs(target_waypoint_index, 2) - x_current[2];
        // auto t_start = std::chrono::high_resolution_clock::now();
        update_and_solve(x_current);
        // auto t_end = std::chrono::high_resolution_clock::now();
        // double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        // time_record(hsy) = elapsed_time_ms;
        integrate_next_states();
        // for(int i=0; i<nu; i++) {
        //     simU(hsy, i) = u_current[i];
        // }
        // for(int ii=0; ii<nx; ii++) {
        //     simX(hsy, ii) = x_current[ii];
        // }
        hsy++;
    }
    // Eigen::VectorXd stats = computeStats(hsy);
    return 0;
}

int Optimizer::update_and_solve(Eigen::Vector3d &i_current_state, bool safety_check, int mode) {
    /*
    * Update the reference trajectory and solve the optimization problem
    * Computed control input is stored in u_current
    * Returns 0 if successful, 1 otherwise
    */
    auto t_start = std::chrono::high_resolution_clock::now(); // debug timer
    if (mode == -1) {
        state_refs_ptr = &state_refs;
    } else if (mode == 0) {
        state_refs_ptr = &left_turn_states;
    } else if (mode == 1) {
        state_refs_ptr = &straight_states;
    } else if (mode == 2) {
        state_refs_ptr = &right_turn_states;
    } else {
        std::cerr << "Invalid mode, proceed with default mode" << std::endl;
        state_refs_ptr = &state_refs;
    }
    if(debug) {
        x_errors(iter) = (*state_refs_ptr)(target_waypoint_index, 0) - i_current_state[0];
        y_errors(iter) = (*state_refs_ptr)(target_waypoint_index, 1) - i_current_state[1];
        yaw_errors(iter) = (*state_refs_ptr)(target_waypoint_index, 2) - i_current_state[2];
        // auto t_start = std::chrono::high_resolution_clock::now();
    }
    int success = find_next_waypoint(target_waypoint_index, i_current_state, safety_check);
    // if(!success) {
    //     u_current[0] = v_ref;
    //     u_current[0] = 0;
    //     std::cout << "WARNING: Optimizer::update_and_solve(): vehicle is stuck. publishing default control input: " << u_current[0] << ", " << u_current[1] << std::endl;
    //     return 0;
    // }
    int idx = target_waypoint_index;
    if (target_waypoint_index >= state_refs.rows()) {
        idx = state_refs.rows() - 1;
    }
    for(int i=0; i<3; i++) {
        x_state[i] = (*state_refs_ptr)(target_waypoint_index, i);
    }
    x_state[3] = 0.0;
    x_state[4] = 0.0;

    // Set the reference trajectory for the optimization problem
    ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", x_state);

    // Set the reference trajectory for next N steps
    for (int j = 0; j < N; ++j) {
        if (target_waypoint_index + j >= (*state_refs_ptr).rows()) { 
            // if the target wpt exceeds the last wpt, set the last wpt as the target wpt
            for(int i=0; i<3; i++) {
                x_state[i] = (*state_refs_ptr)((*state_refs_ptr).rows() - 1, i);
            }
            x_state[3] = 0.0;
            x_state[4] = 0.0;
        } else {
            for (int i = 0; i < 3; ++i) {
                x_state[i] = (*state_refs_ptr)(idx + j, i);
            }
            for (int i = 0; i < 2; ++i) {
                x_state[i + 3] = input_refs(idx + j, i);
                // x_state[i + 3] = input_refs(idx + j, i)*1.5;
            }
        }
        x_state[4] = 0.0; // set steer rate to 0
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref", x_state);
        // double u_max[2] = {2, 0.4};
        // ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, j, "ubu", u_max);
    }
    // std::cout << "x_state: " << x_state[0] << ", " << x_state[1] << ", " << x_state[2] << ", " << x_state[3] << ", " << x_state[4] << std::endl;

    // Set the constraints for the current state
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", i_current_state.data());
    ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", i_current_state.data());

    // Solve the optimization problem
    if(use25) {
        status = mobile_robot_25_acados_solve(acados_ocp_capsule_25);
    } else if(use18) {
        status = mobile_robot_18_acados_solve(acados_ocp_capsule_18);
    } else {
        status = mobile_robot_acados_solve(acados_ocp_capsule);
    }
    if (status != 0) {
        std::cout << "ERROR!!! acados acados_ocp_solver returned status " << status << ". Exiting." << std::endl;
        return 1; 
    }

    // Get the optimal control for the next step
    ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);

    //debug
    double error = (i_current_state[0] - (*state_refs_ptr)(idx, 0)) * (i_current_state[0] - (*state_refs_ptr)(idx, 0)) + (i_current_state[1] - (*state_refs_ptr)(idx, 1)) * (i_current_state[1] - (*state_refs_ptr)(idx, 1));
    if (debug) {
        auto t_end = std::chrono::high_resolution_clock::now();
        double elapsed_time_ms = std::chrono::duration<double, std::milli>(t_end - t_start).count();
        // std::cout << "elapsed time: " << elapsed_time_ms << " ms" << std::endl;
        time_record(iter) = elapsed_time_ms;
        x_errors(iter) = state_refs(target_waypoint_index, 0) - x_real[0];
        y_errors(iter) = state_refs(target_waypoint_index, 1) - x_real[1];
        yaw_errors(iter) = state_refs(target_waypoint_index, 2) - x_real[2];
        for(int i=0; i<nu; i++) {
            simU(iter, i) = u_current[i];
        }
        for(int ii=0; ii<nx; ii++) {
            simX(iter, ii) = i_current_state[ii];
        }
        // if (idx < state_refs.rows()) printf("%d) x_cur: %.3f, %.3f, %.3f, ref: %.3f, %.3f, %.3f, u: %.3f, %.3f, error: %.3f\n", iter, x_current[0], x_current[1], x_current[2], state_refs(idx, 0), state_refs(idx, 1), state_refs(idx, 2), u_current[0], u_current[1], error);
        // printf("u: (%.3f, %.3f)\n", u_current[0], u_current[1]);
        iter++;
    }
    return 0;
}

void Optimizer::integrate_next_states() {
    // Set the current state and control input for the simulation
    sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "x", x_current.data());
    sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "u", u_current);

    // Run the simulation
    int status_s;
    if(use25) {
        status_s  = mobile_robot_25_acados_sim_solve(sim_capsule_25);
    } else {
        status_s  = mobile_robot_acados_sim_solve(sim_capsule);
    }
    if (status_s != ACADOS_SUCCESS) {
        throw std::runtime_error("acados integrator returned status " + std::to_string(status_s) + ". Exiting.");
    }

    // Get the result and update the current state
    sim_out_get(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_out, "x", x_current.data());

    t0 += T;
}

int Optimizer::find_closest_waypoint(int min_index, int max_index) {
    // auto start = std::chrono::high_resolution_clock::now();
    double current_norm = x_current.head(2).squaredNorm();

    double min_distance_sq = std::numeric_limits<double>::max();
    int closest = -1;
    // double second_min_distance_sq = std::numeric_limits<double>::max();
    // int second_closest = -1;

    static int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]

    // if (min_index < 0) min_index = std::max(target_waypoint_index - limit, 0); //0;
    if (min_index < 0) min_index = std::min(last_waypoint_index, static_cast<int>(state_refs.rows()) - 1);
    if (max_index < 0) max_index = std::min(target_waypoint_index + limit, static_cast<int>(state_refs.rows()) - 1); //state_refs.rows() - 1;
    
    // if (min_index < 0) min_index = 0;
    // if (max_index < 0) max_index = static_cast<int>(state_refs.rows()) - 1;

    // for (int i = min_index; i < max_index; ++i) {
    for (int i = max_index; i >= min_index ; --i) {
        double distance_sq = (state_refs.row(i).head(2).squaredNorm() 
                           - 2 * state_refs.row(i).head(2).dot(x_current.head(2))
                           + current_norm); 

        if (distance_sq < min_distance_sq) {
            // second_min_distance_sq = min_distance_sq;
            // second_closest = closest;
            min_distance_sq = distance_sq;
            closest = i;
        }
    }
    // if (std::abs(closest - second_closest) > limit/3) {
    //     //check which one is closer to target_waypoint_index
    //     if (std::abs(closest - target_waypoint_index) > std::abs(second_closest - target_waypoint_index)) {
    //         printf("WARNING: Optimizer::find_closest_waypoint(): found that second closest is closer to target waypoint\n");
    //         closest = second_closest;
    //     }
    // }
    return closest;
}

int Optimizer::find_next_waypoint(int &output_target, Eigen::Vector3d &i_current_state, bool safety_check, int min_index, int max_index) {
    int target = 0;

    static int limit = floor(rdb_circumference / (v_ref * T)); // rdb circumference [m] * wpt density [wp/m]
    static int lookahead = 1;
    if (v_ref > 0.375) lookahead = 1;
    static Eigen::Vector3d last_state = i_current_state;
    double distance_travelled_sq = (i_current_state.head(2) - last_state.head(2)).squaredNorm();
    // static int stuck_count = 0;
    // if (distance_travelled_sq < (v_ref * T * 0.5) * (v_ref * T * 0.5)) {
    //     printf("WARNING: Optimizer::find_next_waypoint(): distance travelled is too small: %.3f\n", std::sqrt(distance_travelled_sq));
    //     stuck_count++;
    // }
    // if (u_current[0] < v_ref * 0.25) {
    //     printf("WARNING: Optimizer::find_next_waypoint(): speed is too low: %.3f\n", u_current[0]);
    //     stuck_count++;
    // }
    last_state = i_current_state;

    static int count = 0;
    closest_waypoint_index = find_closest_waypoint(min_index, max_index);
    double distance_to_current = std::sqrt((state_refs(closest_waypoint_index, 0) - i_current_state[0]) * (state_refs(closest_waypoint_index, 0) - i_current_state[0]) + (state_refs(closest_waypoint_index, 1) - i_current_state[1]) * (state_refs(closest_waypoint_index, 1) - i_current_state[1]));
    if (distance_to_current > 1.2) {
        std::cout << "WARNING: Optimizer::find_next_waypoint(): distance to closest waypoint is too large: " << distance_to_current << std::endl;
        min_index = static_cast<int>(std::max(closest_waypoint_index - distance_to_current * density * 1.2, 0.0));
        closest_waypoint_index = find_closest_waypoint(min_index , max_index);
    }
    // if (stuck_count > 3) {
    //     printf("ERROR: Optimizer::find_next_waypoint(): vehicle is stuck. resetting solver\n");
    //     // reset_solver();
    //     target = closest_waypoint_index + lookahead + 2;
    //     output_target = std::min(target, static_cast<int>((*state_refs_ptr).rows()) - 1);
    //     stuck_count = 0;
    //     return 0;
    // } else
    if (count >= 8) {
        target = closest_waypoint_index + lookahead;
        count = 0;
    } else {
        target = target_waypoint_index + 1;
        count++;
    }
    // std::cout << "closest: " << closest_waypoint_index << ", target: " << target << ", limit: " << limit << ", lookahead: " << lookahead << ", count: " << count << std::endl;

    output_target =  std::min(target, static_cast<int>((*state_refs_ptr).rows()) - 1);
    last_waypoint_index = output_target;
    return 1;
}
int Optimizer::update_current_states(double x, double y, double yaw, Eigen::Vector3d &state, bool safety_check) {
    int success = 1;
    if(closest_waypoint_index < state_refs.rows() && state_refs.rows() > 0) {
        double ref_yaw = state_refs(closest_waypoint_index, 2);
        while (ref_yaw - yaw > M_PI) {
            yaw += 2 * M_PI;
        }
        while (ref_yaw - yaw < -M_PI) {
            yaw -= 2 * M_PI;
        }
    }
    if (safety_check) {
        // double difference_mag_sq = (state[0] - x) * (state[0] - x) + (state[1] - y) * (state[1] - y);
        // if (difference_mag_sq > std::pow(v_ref * T * 5, 2)) {
        //     std::cout << "difference is too large, initial: " << state[0] << ", " << state[1] << ", " << state[2] << ", current: " << x << ", " << y << ", " << yaw << ", norm sq: " << difference_mag_sq << std::endl;
        //     reset_solver();
        //     success = 0;
        // }
    }
    state[0] = x;
    state[1] = y;
    state[2] = yaw;
    return success;
}
// void Optimizer::update_current_states(double* state) {
//     double yaw = state[2];
//     if(target_waypoint_index < state_refs.rows()) {
//         double ref_yaw = state_refs(target_waypoint_index, 2);
//         while (ref_yaw - yaw > M_PI) {
//             yaw += 2 * M_PI;
//         }
//         while (ref_yaw - yaw < -M_PI) {
//             yaw -= 2 * M_PI;
//         }
//     }
//     x_current[0] = state[0];
//     x_current[1] = state[1];
//     x_current[2] = yaw;
// }

Eigen::VectorXd Optimizer::computeStats(int hsy) {
    simX.conservativeResize(iter, Eigen::NoChange);
    simU.conservativeResize(iter, Eigen::NoChange);
    time_record.conservativeResize(iter, Eigen::NoChange);
    printf("average estimation time %f ms \n", time_record.mean());
    printf("max estimation time %f ms \n", time_record.maxCoeff());
    printf("min estimation time %f ms \n", time_record.minCoeff());

    // Calculate averages for speed and steer
    double average_speed = simU.col(0).mean();
    double average_steer = simU.col(1).mean();

    // Calculate differences for speed and steer
    Eigen::MatrixXd deltaU = simU.bottomRows(simU.rows() - 1) - simU.topRows(simU.rows() - 1);
    double average_delta_speed = deltaU.col(0).cwiseAbs().mean();
    double average_delta_steer = deltaU.col(1).cwiseAbs().mean();

    // Output results
    std::cout << "Average speed: " << average_speed << " m/s\n";
    std::cout << "Average steer angle: " << average_steer << " rad\n";
    std::cout << "Average change in speed: " << average_delta_speed << " m/s²\n";
    std::cout << "Average change in steer angle: " << average_delta_steer << " rad/s\n";

    // Calculate average errors
    double average_x_error = x_errors.cwiseAbs().mean();
    double average_y_error = y_errors.cwiseAbs().mean();
    yaw_errors = (yaw_errors.array().sin().binaryExpr(yaw_errors.array().cos(), std::ptr_fun(atan2))).matrix();
    double average_yaw_error = yaw_errors.cwiseAbs().mean();

    // Output error results
    std::cout << "Average x error: " << average_x_error << " m\n";
    std::cout << "Average y error: " << average_y_error << " m\n";
    std::cout << "Average yaw error: " << average_yaw_error << " rad\n";

    // Return the statistics as a vector
    Eigen::VectorXd stats(7);
    stats << average_speed, average_steer, average_delta_speed, average_delta_steer,
                average_x_error, average_y_error, average_yaw_error;
    
    // Eigen::VectorXd stats = computeStats();
    saveToFile(simX, "results/simX.txt");
    saveToFile(simU, "results/simU.txt");
    // saveToFile(time_record, "time_record.txt"); 
    saveToFile(stats, "results/stats.txt");
    saveToFile(state_refs, "results/state_refs.txt");
    return stats;
}

// Helper functions
std::string Optimizer::getSourceDirectory() {
    std::string file_path(__FILE__);  // __FILE__ is the full path of the source file
    size_t last_dir_sep = file_path.rfind('/');  // For Unix/Linux path
    if (last_dir_sep == std::string::npos) {
        last_dir_sep = file_path.rfind('\\');  // For Windows path
    }
    if (last_dir_sep != std::string::npos) {
        return file_path.substr(0, last_dir_sep);  // Extract directory path
    }
    return "";  // Return empty string if path not found
}
template <typename EigenType>
void Optimizer::saveToFile(const EigenType &data, const std::string &filename) {
    std::string dir = getSourceDirectory();
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
Eigen::MatrixXd Optimizer::loadTxt(const std::string &filename) {
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
// int main() {
//     Optimizer controller;
//     controller.run();
//     return 0;
// }
