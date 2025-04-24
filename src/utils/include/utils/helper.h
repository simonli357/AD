#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>

namespace helper {
    inline std::string getSourceDirectory() {
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

    inline std::string d2str(double value, int precision = 3) {
        std::ostringstream out;
        out << std::fixed << std::setprecision(precision) << value;
        return out.str();
    }

    inline double yaw_mod(double& io_yaw, double ref=0) {
        double yaw = io_yaw;
        while (yaw - ref > M_PI) yaw -= 2 * M_PI;
        while (yaw - ref <= -M_PI) yaw += 2 * M_PI;
        io_yaw = yaw;
        return yaw;
    }
    
    inline double compare_yaw(double yaw1, double yaw2) {
        // returns the absolute difference between two yaw angles
        double diff = yaw1 - yaw2;
        diff = yaw_mod(diff);
        return std::abs(diff);
    }
    
    inline const double directions[5] = {0, M_PI / 2, M_PI, 3 * M_PI / 2, 2 * M_PI};
    inline double nearest_direction(double yaw) {
        yaw = yaw_mod(yaw, M_PI);

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
    
    inline int nearest_direction_index(double yaw) {
        yaw = yaw_mod(yaw, M_PI);

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
}

