#pragma once

#include <array>
#include <stdexcept>
#include <string>
#include <vector>

namespace VehicleTunables {
    inline std::array<double, 6> to_realsense_tf_array(
        const std::vector<double>& values,
        const std::string& param_name
    ) {
        if (values.size() != 6) {
            throw std::invalid_argument(param_name + " must contain exactly 6 values: x, y, z, roll, pitch, yaw");
        }
        return {values[0], values[1], values[2], values[3], values[4], values[5]};
    }
}
