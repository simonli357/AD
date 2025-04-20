#pragma once

#include "Database.hpp"
#include <array>

class CameraParams {
  public:
	CameraParams(Database &db);
	CameraParams(CameraParams &&) = default;
	CameraParams(const CameraParams &) = delete;
	CameraParams &operator=(CameraParams &&) = delete;
	CameraParams &operator=(const CameraParams &) = delete;
	~CameraParams() = default;

	Database &db;

	void set_camera_sim_params(const std::array<double, 4> &params);
	void set_camera_real_params(const std::array<double, 4> &params);
	void set_realsense_tf_sim_params(const std::array<double, 6> &params);
	void set_realsense_tf_real_params(const std::array<double, 6> &param);
};
