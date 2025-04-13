#include "Track.hpp"
#include "SplineUtils.hpp"
#include <Eigen/Core>
#include <unsupported/Eigen/Splines>

using namespace Eigen;

std::vector<double> SplineUtils::build_query_normalized(size_t numPoints, int density) {
    std::vector<double> query;
    size_t numSegments = numPoints - 1;
    query.reserve(numSegments * density);
    for (size_t i = 0; i < numSegments; ++i) {
        double t_start = static_cast<double>(i);
        for (int j = 0; j < density; ++j) {
            double t = t_start + static_cast<double>(j) / density;
            query.push_back(t);
        }
    }
    return query;
}

std::vector<Track::Vertex> SplineUtils::interpolate_path(const std::vector<Track::Vertex> &path, int density) {
    if (path.size() < 2) {
        return {};
    }

    size_t num_original = path.size();
    int degree;
    if (num_original == 2) {
        degree = 1;
    } else if (num_original == 3) {
        degree = 2;
    } else {
        degree = 3;
    }

    MatrixXd points(2, num_original);
    for (size_t i = 0; i < num_original; ++i) {
        points(0, i) = path[i].x;
        points(1, i) = path[i].y;
    }

    RowVectorXd knots(num_original);
    for (size_t i = 0; i < num_original; ++i) {
        knots[i] = static_cast<double>(i);
    }

    using SplineType = Spline<double, 2>;
    SplineType spline = SplineFitting<SplineType>::Interpolate(points, degree, knots);

    std::vector<double> query = build_query_normalized(num_original, density);
    std::vector<Track::Vertex> result;
    result.reserve(query.size());

    for (double t : query) {
        size_t segment = static_cast<size_t>(t);
        if (segment >= num_original - 1) {
            segment = num_original - 2;
        }

        Vector2d pt = spline(t);
        Track::Vertex vertex;
        vertex.id = -1; // Assign a default or appropriate ID
        vertex.x = pt.x();
        vertex.y = pt.y();
        vertex.attribute = path[segment].attribute;

        result.push_back(vertex);
    }

    return result;
}
