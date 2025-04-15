#include "utils/FilterUtils.hpp"

void FilterUtils::yaw_filter(std::vector<Vertex> &path, double max_yaw_change) {
    for (size_t i = 1; i < path.size();) {
        double yaw_change = std::fabs(path[i].normal_angle - path[i - 1].normal_angle);
        if (yaw_change > max_yaw_change) {
            path.erase(path.begin() + i);
        } else {
            ++i;
        }
    }
}

void FilterUtils::distance_filter(std::vector<Vertex> &path, double thresh) {
    for (size_t i = 1; i < path.size();) {
        double current_thresh = thresh;
        switch (path[i].attribute) {
            case Track::CROSSWALK:
                current_thresh /= 1.5;
                break;
            case Track::HIGHWAY_LEFT:
                current_thresh *= 1.33;
                break;
            case Track::HIGHWAY_RIGHT:
                current_thresh *= 1.33;
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

double FilterUtils::euclidean_distance(const Vertex &src, const Vertex &dest) {
	double dx = dest.x - src.x;
	double dy = dest.y - src.y;
	return std::sqrt(dx * dx + dy * dy);
}
