#include "utils/constants.h"
#include "queries/CameraQueries.hpp"

void VehicleConstants::init_params(Database &db) {
    db.cam_queries->set_camera_sim_params(CAMERA_PARAMS);
    db.cam_queries->set_camera_real_params(CAMERA_PARAMS_REAL);
    db.cam_queries->set_realsense_tf_sim_params(REALSENSE_TF);
    db.cam_queries->set_realsense_tf_real_params(REALSENSE_TF_REAL);
}
