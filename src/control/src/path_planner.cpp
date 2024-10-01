#include <iostream>
#include <eigen3/Eigen/Dense>
#include <interpolation.h> // Alglib header for spline interpolation
#include <alglibinternal.h> 
#include <cmath>
#include <vector>
#include <optional>
#include <tuple>
#include <fstream>
#include <string>
#include <limits>

Eigen::MatrixXd load(const std::string& file_path) {
        std::cout << "Loading file: " << file_path << std::endl;
        std::ifstream file(file_path);
        if (!file.is_open()) {
            throw std::runtime_error("Unable to open file");
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

std::tuple<Eigen::VectorXd, Eigen::VectorXd, Eigen::MatrixXd> compute_smooth_curvature(
    Eigen::VectorXd& waypoints_x, 
    Eigen::VectorXd& waypoints_y, 
    double smooth_factor = 0.1) 
{
    using namespace alglib;
    int n_points = waypoints_x.size();

    // Step 1: Set up a spline interpolation
    real_1d_array t, x, y;
    t.setlength(n_points);
    x.setlength(n_points);
    y.setlength(n_points);

    for (int i = 0; i < n_points; ++i) {
        t[i] = static_cast<double>(i) / (n_points - 1);
        x[i] = waypoints_x[i];
        y[i] = waypoints_y[i];
    }

    spline1dinterpolant spline_x, spline_y;
    alglib::spline1dbuildcubic(t, x, spline_x);
    alglib::spline1dbuildcubic(t, y, spline_y);

    // Step 2: Get smoothed derivatives of the path
    Eigen::VectorXd dx_dt(n_points), dy_dt(n_points);
    Eigen::VectorXd ddx_dt(n_points), ddy_dt(n_points);
    double v, dv, ddv;
    for (int i = 0; i < n_points; ++i) {
        double ti = static_cast<double>(i) / (n_points - 1);
        
        alglib::spline1ddiff(spline_x, ti, v, dv, ddv);
        dx_dt[i] = dv;
        ddx_dt[i] = ddv;

        alglib::spline1ddiff(spline_y, ti, v, dv, ddv);
        dy_dt[i] = dv;
        ddy_dt[i] = ddv;
    }

    // Step 3: Compute curvature
    Eigen::VectorXd curvature(n_points);
    for (int i = 0; i < n_points; ++i) {
        double denominator = std::pow(dx_dt[i] * dx_dt[i] + dy_dt[i] * dy_dt[i], 1.5);
        curvature[i] = std::abs(dx_dt[i] * ddy_dt[i] - dy_dt[i] * ddx_dt[i]) / denominator;
    }

    // Step 4: Compute tangent
    Eigen::VectorXd tangent_angles(n_points);
    for (int i = 0; i < n_points; ++i) {
        tangent_angles[i] = std::atan2(dy_dt[i], dx_dt[i]);
    }

    // Step 5: Compute normal
    Eigen::MatrixXd normals(n_points, 2);
    for (int i = 0; i < n_points; ++i) {
        double normal_angle = tangent_angles[i] + M_PI / 2;
        normals(i, 0) = std::cos(normal_angle); // dx
        normals(i, 1) = std::sin(normal_angle); // dy
    }

    return std::make_tuple(curvature, tangent_angles, normals);
}

Eigen::MatrixXd interpolate_waypoints(Eigen::MatrixXd& waypoints, int numPoints) {
    std::cout << "Interpolating waypoints from " << waypoints.rows() << " to " << numPoints << std::endl;
    using namespace alglib;

    // Extract x and y coordinates
    std::vector<double> x(waypoints.rows()), y(waypoints.rows());
    for (int i = 0; i < waypoints.rows(); ++i) {
        x[i] = waypoints(i, 0);
        y[i] = waypoints(i, 1);
    }
    std::cout << "x: " << x.size() << ", y: " << y.size() << std::endl;

    alglib::real_1d_array alglib_x, alglib_y;
    alglib_x.setcontent(x.size(), x.data());
    alglib_y.setcontent(y.size(), y.data());

    std::cout << "alglib_x: " << alglib_x.length() << ", alglib_y: " << alglib_y.length() << std::endl;

    alglib::spline1dinterpolant s;
    // Now call the function with the correct alglib types
    alglib::spline1dbuildcubic(alglib_x, alglib_y, s);
    // spline1dbuildcubic(x.data(), y.data(), waypoints.rows(), 0, 0.0, 0, 0.0, s);

    std::cout << "created spline" << std::endl;

    // Generate new waypoints
    Eigen::MatrixXd newWaypoints(numPoints, 2);
    double step = 1.0 / (numPoints - 1);
    for (int i = 0; i < numPoints; ++i) {
        double u = i * step;
        newWaypoints(i, 0) = spline1dcalc(s, u);
        newWaypoints(i, 1) = spline1dcalc(s, u);
    }

    std::cout << "Generated new waypoints" << std::endl;

    return newWaypoints;
}

Eigen::MatrixXd filter_waypoints(Eigen::MatrixXd& waypoints, double threshold) {
    if (waypoints.rows() == 0) return Eigen::MatrixXd(0, 2);

    std::vector<Eigen::RowVector2d> temp_filtered;
    temp_filtered.push_back(waypoints.row(0));

    for (int i = 1; i < waypoints.rows(); ++i) {
        if ((waypoints.row(i) - waypoints.row(i - 1)).norm() >= threshold) {
            temp_filtered.push_back(waypoints.row(i));
        }
    }

    // Converting the std::vector to Eigen::MatrixXd
    Eigen::MatrixXd filtered_waypoints(temp_filtered.size(), 2);
    for (size_t i = 0; i < temp_filtered.size(); ++i) {
        filtered_waypoints.row(i) = temp_filtered[i];
    }

    return filtered_waypoints;
}

Eigen::VectorXd smooth_yaw_angles(Eigen::VectorXd& yaw_angles) {
    if (yaw_angles.size() == 0) return Eigen::VectorXd(0);

    Eigen::VectorXd diffs = yaw_angles.tail(yaw_angles.size() - 1) - yaw_angles.head(yaw_angles.size() - 1);
    
    // Adjusting differences greater than pi
    for (int i = 0; i < diffs.size(); ++i) {
        if (diffs[i] > M_PI) diffs[i] -= 2 * M_PI;
        else if (diffs[i] < -M_PI) diffs[i] += 2 * M_PI;
    }

    // Compute the smoothed yaw angles
    Eigen::VectorXd smooth_yaw(yaw_angles.size());
    smooth_yaw[0] = yaw_angles[0];
    for (int i = 1; i < yaw_angles.size(); ++i) {
        smooth_yaw[i] = smooth_yaw[i - 1] + diffs[i - 1];
    }

    return smooth_yaw;
}

class Path {
public:
    double v_ref, density, region_of_acceptance, T;
    std::string name;
    int N, num_waypoints;
    Eigen::MatrixXd waypoints;
    bool x0_initialized;
    Eigen::Vector3d x0;
    Eigen::VectorXd kappa, wp_theta, wp_normals, v_refs, steer_ref;
    Eigen::MatrixXd state_refs, input_refs;

    Path(double v_ref, int N, double T, const std::optional<Eigen::Vector3d>& x0 = std::nullopt, const std::string& name = "speedrun")
        : v_ref(v_ref), N(N), T(T), name(name), x0_initialized(false)
    {
        std::string path = "/home/simonli/Simulator/src/mpc/scripts/";
        // Load runs from files
        std::vector<Eigen::MatrixXd> runs = {load(path+"run1.txt"), load(path+"run2.txt"), load(path+"run3.txt"), load(path+"run4.txt"), load(path+"run5.txt")};
        
        if(x0) {
            std::cout << "x0: " << x0.value() << std::endl;
            this->x0 = x0.value(); 
            x0_initialized = true;
            runs = findAndModifyClosestRun(runs, x0.value());
        }

        density = 1.0 / std::abs(v_ref) / T; // wp/m
        region_of_acceptance = 0.05 / 10 * density;
        std::cout << "Density: " << density << ", region_of_acceptance: " << region_of_acceptance << std::endl;
        // Calculate path lengths and interpolate waypoints
        for (size_t i = 0; i < runs.size(); ++i) {
            double length = calculatePathLength(runs[i]);
            runs[i] = interpolate_waypoints(runs[i], static_cast<int>(std::ceil(length * density)));
        }

        // Combine all runs into a single set of waypoints
        waypoints = concatenateRuns(runs);
        std::cout << "Waypoint shape: " << waypoints.rows() << " x " << waypoints.cols() << std::endl;
        waypoints = filter_waypoints(waypoints, 0.01);

        // Calculate the total path length
        double total_path_length = calculatePathLength(waypoints);
        std::cout << "Total path length: " << total_path_length << std::endl;

        Eigen::VectorXd waypoints_x = waypoints.col(0);
        Eigen::VectorXd waypoints_y = waypoints.col(1);
        num_waypoints = waypoints_x.rows();
        std::cout << "num_waypoints: " << num_waypoints << std::endl;

        std::tie(kappa, wp_theta, wp_normals) = compute_smooth_curvature(waypoints_x, waypoints_y);
        v_refs = v_ref / (1 + kappa.array().abs()) * (1 + kappa.array().abs());

        // Handle ramp waypoints
        int num_ramp_waypoints = std::min(static_cast<int>(1 * density), num_waypoints);
        for (int i = 0; i < num_ramp_waypoints; ++i) {
            v_refs[i] = i * v_ref / num_ramp_waypoints;
        }
        v_refs.tail(2).setZero(); // stop at the end

        double k_steer = 0; // Adjust as needed
        steer_ref = k_steer * kappa;
    }

private:
    double calculatePathLength(Eigen::MatrixXd& run) {
        if (run.rows() < 2) {
            return 0.0;
        }

        double length = 0.0;
        for (int i = 1; i < run.rows(); ++i) {
            length += (run.row(i) - run.row(i - 1)).norm();
        }

        std::cout << "Path length: " << length << std::endl;
        return length;
    }

    Eigen::MatrixXd concatenateRuns(std::vector<Eigen::MatrixXd>& runs) {
        if (runs.empty()) {
            return Eigen::MatrixXd(0, 0);
        }

        // Calculate the total number of rows
        int total_rows = 0;
        for (auto& run : runs) {
            total_rows += run.rows();
        }

        // Assuming all runs have the same number of columns
        int cols = runs.front().cols();
        Eigen::MatrixXd concatenated(total_rows, cols);
        int current_row = 0;

        for (auto& run : runs) {
            concatenated.block(current_row, 0, run.rows(), cols) = run;
            current_row += run.rows();
        }

        return concatenated;
    }
    std::vector<Eigen::MatrixXd> findAndModifyClosestRun(const std::vector<Eigen::MatrixXd>& runs, const Eigen::Vector3d& x0) {
        double min_distance = std::numeric_limits<double>::infinity();
        int min_distance_run_index = -1;
        int min_distance_point_index = -1;

        for (size_t i = 0; i < runs.size(); ++i) {
            const auto& run = runs[i];
            for (int j = 0; j < run.cols(); ++j) {
                double distance = (run.col(j).head<2>() - x0.head<2>()).norm();
                if (distance < min_distance) {
                    min_distance = distance;
                    min_distance_run_index = static_cast<int>(i);
                    min_distance_point_index = j;
                }
            }
        }

        std::vector<Eigen::MatrixXd> modified_runs = runs;
        if (min_distance_run_index != -1) {
            Eigen::MatrixXd& closest_run = modified_runs[min_distance_run_index];
            // Insert x0 into the closest run before the closest waypoint
            Eigen::MatrixXd modified_run(closest_run.rows(), closest_run.cols() + 1);
            modified_run << closest_run.leftCols(min_distance_point_index), x0.head<2>(), closest_run.rightCols(closest_run.cols() - min_distance_point_index);

            // Eliminate waypoints before x0
            modified_run = modified_run.rightCols(modified_run.cols() - min_distance_point_index);

            // Update the list of runs
            modified_runs[min_distance_run_index] = modified_run;
        }

        return modified_runs;
    }
};

int main() {
    // Example usage
    Eigen::Vector3d x0(3, 2, 0);
    Path path(1.0, 100, 1.0, x0, "speedrun");

    // Perform other operations with 'path'
}
