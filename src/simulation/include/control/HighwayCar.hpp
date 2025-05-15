#include "Car.hpp"

class HighwayCar : public Car {
  public:
	HighwayCar(TrafficManager &traffic_manager, ros::NodeHandle &nh, double vref, std::string car_name);
	HighwayCar(HighwayCar &&) = delete;
	HighwayCar(const HighwayCar &) = delete;
	HighwayCar &operator=(HighwayCar &&) = delete;
	HighwayCar &operator=(const HighwayCar &) = delete;
	~HighwayCar() = default;

	void start() override;

  protected:
	void run() override;
	void plan_path() override;
};
