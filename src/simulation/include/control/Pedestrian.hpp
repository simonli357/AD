#include "PathPlanner.hpp"
#include "TrafficManager.hpp"
#include <ros/node_handle.h>
#include <string>

class Pedestrian {
  public:
	Pedestrian(TrafficManager &traffic_manager, ros::NodeHandle &nh, std::string name);
	Pedestrian(Pedestrian &&) = default;
	Pedestrian(const Pedestrian &) = default;
	Pedestrian &operator=(Pedestrian &&) = delete;
	Pedestrian &operator=(const Pedestrian &) = delete;
	~Pedestrian() = default;

	using Vertex = PathPlanner::Vertex;

	void run(Vertex intersection);

  private:
	TrafficManager &traffic_manager;
    ros::NodeHandle &nh;
	std::string pedestrian_name;

	void move_pedestrian_to(double x, double y, double yaw);
};
