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

	std::array<double, 4> fetch_camera_sim_params();
	std::array<double, 4> fetch_camera_real_params();
};
