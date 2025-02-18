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
#include "acados_sim_solver_mobile_robot_25.h"
#include "acados_solver_mobile_robot_25.h"
#include "acados_sim_solver_mobile_robot_32.h"
#include "acados_solver_mobile_robot_32.h"
#include "acados_sim_solver_mobile_robot_25_beta.h"
#include "acados_solver_mobile_robot_25_beta.h"
#include "acados_sim_solver_mobile_robot_32_beta.h"
#include "acados_solver_mobile_robot_32_beta.h"
#include "utils/constants.h"

class MPC {
public:
    MPC(double T, int N, double v_ref, bool use_beta):
        T(T), N(N), v_ref(v_ref), use_beta(use_beta)
    {
        std::cout << "MPC Constructor" << std::endl;
        v_ref_int = static_cast<int>(v_ref * 100); // convert to cm/s
        status = 0; // Assuming 0 is a default 'no error' state

        if(v_ref_int == 25) {
            std::cout << "reference speed is 25 cm/s" << std::endl;
            use25 = true;
            if (use_beta) {
                acados_ocp_capsule_25_beta = mobile_robot_25_beta_acados_create_capsule();
                status = mobile_robot_25_beta_acados_create(acados_ocp_capsule_25_beta);
                if (status) {
                    printf("mobile_robot_25_beta_acados_create() returned status %d. Exiting.\n", status);
                    exit(1);
                }
                sim_capsule_25_beta = mobile_robot_25_beta_acados_sim_solver_create_capsule();
                status = mobile_robot_25_beta_acados_sim_create(sim_capsule_25_beta);
                mobile_robot_sim_config = mobile_robot_25_beta_acados_get_sim_config(sim_capsule_25_beta);
                mobile_robot_sim_dims = mobile_robot_25_beta_acados_get_sim_dims(sim_capsule_25_beta);
                mobile_robot_sim_in = mobile_robot_25_beta_acados_get_sim_in(sim_capsule_25_beta);
                mobile_robot_sim_out = mobile_robot_25_beta_acados_get_sim_out(sim_capsule_25_beta);
                if (status) {
                    printf("acados_create() simulator returned status %d. Exiting.\n", status);
                    exit(1);
                }
                nlp_config = mobile_robot_25_beta_acados_get_nlp_config(acados_ocp_capsule_25_beta);
                nlp_dims = mobile_robot_25_beta_acados_get_nlp_dims(acados_ocp_capsule_25_beta);
                nlp_in = mobile_robot_25_beta_acados_get_nlp_in(acados_ocp_capsule_25_beta);
                nlp_out = mobile_robot_25_beta_acados_get_nlp_out(acados_ocp_capsule_25_beta);
            } else {
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
            }
        } else if(v_ref_int == 32) {
            std::cout << "reference speed is 32 cm/s" << std::endl;
            use32 = true;
            use25 = false;
            if (use_beta) {
                acados_ocp_capsule_32_beta = mobile_robot_32_beta_acados_create_capsule();
                status = mobile_robot_32_beta_acados_create(acados_ocp_capsule_32_beta);
                if (status) {
                    printf("mobile_robot_32_beta_acados_create() returned status %d. Exiting.\n", status);
                    exit(1);
                }
                sim_capsule_32_beta = mobile_robot_32_beta_acados_sim_solver_create_capsule();
                status = mobile_robot_32_beta_acados_sim_create(sim_capsule_32_beta);
                mobile_robot_sim_config = mobile_robot_32_beta_acados_get_sim_config(sim_capsule_32_beta);
                mobile_robot_sim_dims = mobile_robot_32_beta_acados_get_sim_dims(sim_capsule_32_beta);
                mobile_robot_sim_in = mobile_robot_32_beta_acados_get_sim_in(sim_capsule_32_beta);
                mobile_robot_sim_out = mobile_robot_32_beta_acados_get_sim_out(sim_capsule_32_beta);
                if (status) {
                    printf("acados_create() simulator returned status %d. Exiting.\n", status);
                    exit(1);
                }
                nlp_config = mobile_robot_32_beta_acados_get_nlp_config(acados_ocp_capsule_32_beta);
                nlp_dims = mobile_robot_32_beta_acados_get_nlp_dims(acados_ocp_capsule_32_beta);
                nlp_in = mobile_robot_32_beta_acados_get_nlp_in(acados_ocp_capsule_32_beta);
                nlp_out = mobile_robot_32_beta_acados_get_nlp_out(acados_ocp_capsule_32_beta);
            } else {
                acados_ocp_capsule_32 = mobile_robot_32_acados_create_capsule();
                status = mobile_robot_32_acados_create(acados_ocp_capsule_32);
                if (status) {
                    printf("mobile_robot_32_acados_create() returned status %d. Exiting.\n", status);
                    exit(1);
                }
                sim_capsule_32 = mobile_robot_32_acados_sim_solver_create_capsule();
                status = mobile_robot_32_acados_sim_create(sim_capsule_32);
                mobile_robot_sim_config = mobile_robot_32_acados_get_sim_config(sim_capsule_32);
                mobile_robot_sim_dims = mobile_robot_32_acados_get_sim_dims(sim_capsule_32);
                mobile_robot_sim_in = mobile_robot_32_acados_get_sim_in(sim_capsule_32);
                mobile_robot_sim_out = mobile_robot_32_acados_get_sim_out(sim_capsule_32);
                if (status) {
                    printf("acados_create() simulator returned status %d. Exiting.\n", status);
                    exit(1);
                }
                nlp_config = mobile_robot_32_acados_get_nlp_config(acados_ocp_capsule_32);
                nlp_dims = mobile_robot_32_acados_get_nlp_dims(acados_ocp_capsule_32);
                nlp_in = mobile_robot_32_acados_get_nlp_in(acados_ocp_capsule_32);
                nlp_out = mobile_robot_32_acados_get_nlp_out(acados_ocp_capsule_32);
            }
        } else {
            std::cerr << "Invalid reference speed, please use 18, 25 or 32" << std::endl;
            exit(1);
        }

        // Setting problem dimensions
        N = nlp_dims->N;
        nx = *nlp_dims->nx;
        nu = *nlp_dims->nu;

        x_state[0] = 0.0;
        x_state[1] = 0.0;
        x_state[2] = 0.0;
        x_state[3] = 0.0;
        x_state[4] = 0.0;
        x_state[5] = 0.0;
        x_state[6] = 0.0;
        u_current[0] = 0.0;
        u_current[1] = 0.0;
    }
    MPC(): MPC(0.1, 40, 0.3, true) {}
    ~MPC() {
        free(acados_ocp_capsule_25);
        free(sim_capsule_25);
        free(acados_ocp_capsule_32);
        free(sim_capsule_32);
        free(acados_ocp_capsule_25_beta);
        free(sim_capsule_25_beta);
        free(acados_ocp_capsule_32_beta);
        free(sim_capsule_32_beta);
        free(mobile_robot_sim_config);
        free(mobile_robot_sim_dims);
        free(mobile_robot_sim_in);
        free(mobile_robot_sim_out);
        free(nlp_config);
        free(nlp_dims);
        free(nlp_in);
        free(nlp_out);
    }

    int status; // acados operation state
    double x_state[7];
    double u_current[2];
    int N, nx, nu, iter = 0;
    int v_ref_int;
    bool use25 = true;
    bool use32 = false;
    bool use_beta = true;
    double v_ref, t0, T;
   
    mobile_robot_25_solver_capsule *acados_ocp_capsule_25;
    mobile_robot_25_sim_solver_capsule *sim_capsule_25;
    mobile_robot_32_solver_capsule *acados_ocp_capsule_32;
    mobile_robot_32_sim_solver_capsule *sim_capsule_32;
    mobile_robot_25_beta_solver_capsule *acados_ocp_capsule_25_beta;
    mobile_robot_25_beta_sim_solver_capsule *sim_capsule_25_beta;
    mobile_robot_32_beta_solver_capsule *acados_ocp_capsule_32_beta;
    mobile_robot_32_beta_sim_solver_capsule *sim_capsule_32_beta;

    sim_config *mobile_robot_sim_config;
    void *mobile_robot_sim_dims;
    sim_in *mobile_robot_sim_in;
    sim_out *mobile_robot_sim_out;
    ocp_nlp_config *nlp_config;
    ocp_nlp_dims *nlp_dims;
    ocp_nlp_in *nlp_in;
    ocp_nlp_out *nlp_out;

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
    
    int reset_solver() {
        int reset_status;
        if (use_beta) {
            if(use25) {
                reset_status = mobile_robot_25_beta_acados_reset(acados_ocp_capsule_25_beta, 1);
            } else if(use32) {
                reset_status = mobile_robot_32_beta_acados_reset(acados_ocp_capsule_32_beta, 1);
            }
        } else {
            if(use25) {
                reset_status = mobile_robot_25_acados_reset(acados_ocp_capsule_25, 1);
            } else if(use32) {
                reset_status = mobile_robot_32_acados_reset(acados_ocp_capsule_32, 1);
            }
        }
        return reset_status;
    }
    
    int solve(const Eigen::Block<Eigen::MatrixXd>& state_refs, const Eigen::Block<Eigen::MatrixXd>& input_refs, Eigen::Vector3d &i_current_state) {
        /*
        * Update the reference trajectory and solve the optimization problem
        * Computed control input is stored in u_current
        * Returns 0 if successful, 1 otherwise
        */
        
        int idx = 0;
        // --- Set terminal cost (stage N) ---
        static double x_ref_terminal[3] = {0.0};
        for(int i=0; i<3; i++) {
            x_ref_terminal[i] = state_refs(0, i);
        }
        x_ref_terminal[3] = 0.0; // v ref is 0
        x_ref_terminal[4] = 0.0; // steer is 0

        // Set the reference trajectory for the optimization problem
        ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, N, "yref", x_ref_terminal);

        // Set the reference trajectory for next N steps
        for (int j = 0; j < N; ++j) {
            if (j >= state_refs.rows()) { 
                // if the target wpt exceeds the last wpt, set the last wpt as the target wpt
                for(int i=0; i<3; i++) {
                    x_state[i] = state_refs(state_refs.rows() - 1, i);
                }
                x_state[3] = 0.0;
                x_state[4] = 0.0;
            } else {
                for (int i = 0; i < 3; ++i) {
                    x_state[i] = state_refs(idx + j, i);
                }
                for (int i = 0; i < 2; ++i) {
                    x_state[i + 3] = input_refs(idx + j, i);
                }
            }
            x_state[4] = 0.0; // set steer reference to 0
            // Delta_u terms (penalize change from previous control)
            x_state[5] = 0.0; // delta_v_ref (always 0)
            x_state[6] = 0.0; // delta_steer_ref (always 0)
            ocp_nlp_cost_model_set(nlp_config, nlp_dims, nlp_in, j, "yref", x_state);
            if (use_beta) {
                if (use25) {
                    mobile_robot_25_beta_acados_update_params(acados_ocp_capsule_25_beta, j, u_current, 2);
                } else if (use32) {
                    mobile_robot_32_beta_acados_update_params(acados_ocp_capsule_32_beta, j, u_current, 2);
                }
            } else {
                if (use25) {
                    mobile_robot_25_acados_update_params(acados_ocp_capsule_25, j, u_current, 2);
                } else if (use32) {
                    mobile_robot_32_acados_update_params(acados_ocp_capsule_32, j, u_current, 2);
                }
            }
        }

        // Set the constraints for the current state
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "lbx", i_current_state.data());
        ocp_nlp_constraints_model_set(nlp_config, nlp_dims, nlp_in, 0, "ubx", i_current_state.data());

        // Solve the optimization problem
        if (use_beta) {
            if (use25) {
                status = mobile_robot_25_beta_acados_solve(acados_ocp_capsule_25_beta);
            } else if (use32) {
                status = mobile_robot_32_beta_acados_solve(acados_ocp_capsule_32_beta);
            }
        } else {
            if(use25) {
                status = mobile_robot_25_acados_solve(acados_ocp_capsule_25);
            } else if(use32) {
                status = mobile_robot_32_acados_solve(acados_ocp_capsule_32);
            }
        }
        if (status != 0) {
            std::cout << "ERROR!!! acados acados_ocp_solver returned status " << status << ". Exiting." << std::endl;
            return 1; 
        }

        // get the optimal control for the next step
        ocp_nlp_out_get(nlp_config, nlp_dims, nlp_out, 0, "u", &u_current);

        return 0;
    }
    void integrate_next_states(Eigen::Vector3d &io_x_current) {
        // Set the current state and control input for the simulation
        sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "x", io_x_current.data());
        sim_in_set(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_in, "u", u_current);

        // Run the simulation
        int status_s;
        if (use_beta) {
            if(use25) {
                status_s  = mobile_robot_25_beta_acados_sim_solve(sim_capsule_25_beta);
            } else if(use32) {
                status_s  = mobile_robot_32_beta_acados_sim_solve(sim_capsule_32_beta);
            }
        } else {
            if(use25) {
                status_s  = mobile_robot_25_acados_sim_solve(sim_capsule_25);
            } else if(use32) {
                status_s  = mobile_robot_32_acados_sim_solve(sim_capsule_32);
            }
        }
        if (status_s != ACADOS_SUCCESS) {
            throw std::runtime_error("acados integrator returned status " + std::to_string(status_s) + ". Exiting.");
        }

        // Get the result and update the current state
        sim_out_get(mobile_robot_sim_config, mobile_robot_sim_dims, mobile_robot_sim_out, "x", io_x_current.data());

        t0 += T;
    }
};

#endif // OPTIMIZER_HPP