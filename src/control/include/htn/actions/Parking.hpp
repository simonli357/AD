#pragma once

#include "htn/Action.hpp"
#include <unordered_map>

class Parking : public Action {
  public:
	Parking(World &world, std::unordered_map<PRIMITIVES, ValueType> &current_state);
	Parking(Parking &&) = delete;
	Parking(const Parking &) = delete;
	Parking &operator=(Parking &&) = delete;
	Parking &operator=(const Parking &) = delete;
	~Parking();

	void execute() override;

  private:
	void orientation_follow(double orientation, double speed = -2);
	int parking_maneuver_hardcode(bool right = true, bool exit = false, double rate_val = 20, double initial_y_error = 0, double initial_yaw_error = 0);
	int maneuver_hardcode(const Eigen::VectorXd &targets, const Eigen::VectorXd &steerings, const Eigen::VectorXd &speeds, const Eigen::VectorXd &thresholds, double rate_val = 20);
};
