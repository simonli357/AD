#pragma once

#include "HighwayManager.hpp"
#include "control/Car.hpp"
#include <string>

class HighwayCar : public Car {
  public:
	HighwayCar(void *traffic_manager, ros::NodeHandle &nh, double vref, const std::string &car_name, const std::string &path_name);
	HighwayCar(HighwayCar &&) = delete;
	HighwayCar(const HighwayCar &) = delete;
	HighwayCar &operator=(HighwayCar &&) = delete;
	HighwayCar &operator=(const HighwayCar &) = delete;
	~HighwayCar() = default;

	void start() override;

  protected:
	void follow_path();
	void get_path();

  private:
	std::string car_path;
	bool start_car = false;

	void check_if_can_start(double car_x, double car_y, size_t idx);
};
