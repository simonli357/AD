#include "utils/PathUtils.hpp"

void PathUtils::yaw_filter(std::vector<Vertex> &path, double max_yaw_change) {
    for (size_t i = 1; i < path.size();) {
        double yaw_change = std::fabs(path[i].tangent_angle - path[i - 1].tangent_angle);
        if (yaw_change > max_yaw_change) {
            path.erase(path.begin() + i);
        } else {
            ++i;
        }
    }
}

void PathUtils::smooth_yaw(std::vector<Vertex> &path) {
    if (path.size() <= 1) {
        return;
    }
    std::vector<double> yaw_angles;
    yaw_angles.reserve(path.size());
    for (const auto& vertex : path) {
        yaw_angles.push_back(vertex.tangent_angle);
    }
    std::vector<double> diffs;
    diffs.reserve(path.size() - 1);
    for (size_t i = 1; i < yaw_angles.size(); ++i) {
        diffs.push_back(yaw_angles[i] - yaw_angles[i-1]);
    }
    const double threshold = 0.8 * M_PI;
    for (auto& diff : diffs) {
        if (diff > threshold) {
            diff -= 2 * M_PI;
        } else if (diff < -threshold) {
            diff += 2 * M_PI;
        }
    }
    std::vector<double> smooth_yaw;
    smooth_yaw.reserve(path.size());
    smooth_yaw.push_back(yaw_angles[0]);
    for (size_t i = 0; i < diffs.size(); ++i) {
        smooth_yaw.push_back(smooth_yaw[i] + diffs[i]);
    }
    for (size_t i = 0; i < path.size(); ++i) {
        path[i].tangent_angle = smooth_yaw[i];
    }
}

void PathUtils::distance_filter(std::vector<Vertex> &path, double thresh, double hw_density_factor, double cw_density_factor) {
    for (size_t i = 1; i < path.size();) {
        double current_thresh = thresh;
        switch (path[i].attribute) {
            case Track::CROSSWALK:
                current_thresh /= cw_density_factor;
                break;
            case Track::HIGHWAY_LEFT:
                current_thresh *= hw_density_factor;
                break;
            case Track::HIGHWAY_RIGHT:
                current_thresh *= hw_density_factor;
                break;
            default:
                break;
        }

        double dist = euclidean_distance(path[i], path[i - 1]);
        if (dist > current_thresh) {
            path.erase(path.begin() + i);
        } else {
            ++i;
        }
    }
}

void PathUtils::compute_speeds(std::vector<Vertex> &path, double vref, double density, double hw_density_factor, double cw_density_factor) {
    for (auto &v : path) {
        switch (v.attribute) {
            case Track::CROSSWALK:
                v.vref = vref /= cw_density_factor;
                break;
            case Track::HIGHWAY_LEFT:
                v.vref = vref *= hw_density_factor;
                break;
            case Track::HIGHWAY_RIGHT:
                v.vref = vref *= hw_density_factor;
                break;
            default:
                v.vref = vref;
                break;
        }
    }
    size_t num_ramp_points = std::min(static_cast<size_t>(density), path.size());
    if (num_ramp_points > 1) {
        for (size_t i = 0; i < num_ramp_points; i++) {
            double scaling = static_cast<double>(i) / (num_ramp_points - 1);
            path[i].vref = scaling * vref;
        }
    } 
    if (path.size() > 1) {
        path[0].vref = 0;
        path[1].vref = 0;
    } else if (path.size() == 1) {
        path[0].vref = 0;
    }
}

double PathUtils::euclidean_distance(const Vertex &src, const Vertex &dest) {
	double dx = dest.x - src.x;
	double dy = dest.y - src.y;
	return std::sqrt(dx * dx + dy * dy);
}
