#pragma once

#include <iostream>
#include <iomanip>
#include <sstream>
#include <cmath>
#include <Eigen/Dense>
#include <fstream>

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

	inline bool get_min_object_index(const Eigen::Vector2d &estimated_sign_pose, const std::vector<std::vector<double>> &EMPIRICAL_POSES, int &o_index, double &o_min_error_sq, double threshold) {
		int min_index = 0;
		double min_error_sq = 1000;
		// utils.debug("sign_based_relocalization(): estimated sign pose: (" + std::to_string(estimated_sign_pose[0]) + ", " + std::to_string(estimated_sign_pose[1]) + ")", 5);
		for (std::size_t i = 0; i < EMPIRICAL_POSES.size(); ++i) {
			double error_sq = std::pow(estimated_sign_pose[0] - EMPIRICAL_POSES[i][0], 2) + std::pow(estimated_sign_pose[1] - EMPIRICAL_POSES[i][1], 2);
			// std::cout << "object pose: (" << EMPIRICAL_POSES[i][0] << ", " << EMPIRICAL_POSES[i][1] << ", " << EMPIRICAL_POSES[i][2] << "), error: " << std::sqrt(error_sq) << std::endl;
			if (error_sq < min_error_sq) {
				min_error_sq = error_sq;
				min_index = static_cast<int>(i);
			}
		}
		// std::cout << "closest object pose: (" << EMPIRICAL_POSES[min_index][0] << ", " << EMPIRICAL_POSES[min_index][1] << ", " << EMPIRICAL_POSES[min_index][2] << "), error: " << std::sqrt(min_error_sq)
		// << std::endl;
		if (min_error_sq > threshold * threshold) {
			o_index = min_index;
			o_min_error_sq = min_error_sq;
			return false;
		} else {
			o_index = min_index;
			o_min_error_sq = min_error_sq;
			return true;
		}
	}

    inline bool get_min_object_index(
        const Eigen::Vector2d               &estimated_sign_pose,
        const std::vector<Eigen::Vector2d>  &EMPIRICAL_POSES,
        int                                 &o_index,
        double                              &o_min_error_sq,
        double                               threshold
    ) {
        // initialize to something large
        int    min_index    = -1;
        double min_error_sq = std::numeric_limits<double>::infinity();

        for (std::size_t i = 0; i < EMPIRICAL_POSES.size(); ++i) {
            double error_sq = (estimated_sign_pose - EMPIRICAL_POSES[i]).squaredNorm();
            if (error_sq < min_error_sq) {
                min_error_sq = error_sq;
                min_index    = static_cast<int>(i);
            }
        }

        // write outputs
        o_index        = min_index;
        o_min_error_sq = min_error_sq;

        // return whether we're within threshold
        return (min_error_sq <= threshold * threshold);
    }

	template <typename EigenType> inline void saveToFile(const EigenType &data, const std::string &filename) {
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
	inline Eigen::MatrixXd loadTxt(const std::string &filename) {
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

		return Eigen::Map<Eigen::Matrix<double, Eigen::Dynamic, Eigen::Dynamic, Eigen::RowMajor>>(matrixEntries.data(), numRows, numCols);
	}
}

