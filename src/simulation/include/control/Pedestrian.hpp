#include "PathPlanner.hpp"
#include "TrafficManager.hpp"
#include <ros/node_handle.h>
#include <string>

class Pedestrian {
  public:
	Pedestrian(TrafficManager &traffic_manager, ros::NodeHandle &nh, std::string name);
	Pedestrian(Pedestrian &&) = default;
	Pedestrian(const Pedestrian &) = delete;
	Pedestrian &operator=(Pedestrian &&) = delete;
	Pedestrian &operator=(const Pedestrian &) = delete;
	~Pedestrian() = default;

	using Vertex = PathPlanner::Vertex;

	void run();
	void stop();

  private:
	double gazebo_z = 0;
	bool alive = true;

	TrafficManager &traffic_manager;
	ros::NodeHandle &nh;
	ros::Publisher teleport_pub;
	std::string pedestrian_name;

	void setup();
	void move_pedestrian_to(double x, double y, double yaw);
	void hide_pedestrian();
};
