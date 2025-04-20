#include "utils/constants.h"
#include "queries/CameraParams.hpp"

void VehicleConstants::init_params(Database &db) {
    CAMERA_PARAMS = db.cam_queries->fetch_camera_sim_params();
    CAMERA_PARAMS_REAL = db.cam_queries->fetch_camera_real_params();
}
