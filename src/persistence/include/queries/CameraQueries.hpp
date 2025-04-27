#pragma once

#include "Database.hpp"
#include <array>

class CameraQueries {
  public:
	CameraQueries(Database &db);
	CameraQueries(CameraQueries &&) = default;
	CameraQueries(const CameraQueries &) = delete;
	CameraQueries &operator=(CameraQueries &&) = delete;
	CameraQueries &operator=(const CameraQueries &) = delete;
	~CameraQueries() = default;

	Database &db;

	void set_camera_sim_params(const std::array<double, 4> &params);
	void set_camera_real_params(const std::array<double, 4> &params);
	void set_realsense_tf_sim_params(const std::array<double, 6> &params);
	void set_realsense_tf_real_params(const std::array<double, 6> &param);
};
