#pragma once

#include "PathPlanner.hpp"
#include "TrafficManager.hpp"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/spin_rw_mutex.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>

class HighwayCar;
class Pedestrian;

class SpeedCurveManager : public TrafficManager {
  public:
	SpeedCurveManager(ros::NodeHandle &nh, ros::ServiceClient &client, double car_speed);
	SpeedCurveManager(SpeedCurveManager &&) = delete;
	SpeedCurveManager(const SpeedCurveManager &) = delete;
	SpeedCurveManager &operator=(SpeedCurveManager &&) = delete;
	SpeedCurveManager &operator=(const SpeedCurveManager &) = delete;
	~SpeedCurveManager();

	using Graph = PathPlanner::Graph;
	using Vertex = PathPlanner::Vertex;
	using Edge = PathPlanner::Edge;
	using VD = boost::graph_traits<Graph>::vertex_descriptor;
	using ATTR = Track::ATTRIBUTE;

	void spawn_ego_car() override;
	void initialize() override;
	void stop_cars() override;

  private:
	double car_speed = 0.1;
};
