#pragma once

#include "PathPlanner.hpp"
#include "TrafficManager.hpp"
#include <gazebo_msgs/ModelStates.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>
#include <ros/node_handle.h>
#include <ros/service_client.h>
#include <ros/subscriber.h>
#include <tbb/task_arena.h>
#include <tbb/task_group.h>
#include <tbb/concurrent_hash_map.h>
#include <tbb/spin_rw_mutex.h>

class HighwayCar;
class Pedestrian;

class HighwayManager : public TrafficManager {
  public:
	HighwayManager(ros::NodeHandle &nh, ros::ServiceClient &client);
	HighwayManager(HighwayManager &&) = delete;
	HighwayManager(const HighwayManager &) = delete;
	HighwayManager &operator=(HighwayManager &&) = delete;
	HighwayManager &operator=(const HighwayManager &) = delete;
	~HighwayManager();

	using Graph = PathPlanner::Graph;
	using Vertex = PathPlanner::Vertex;
	using Edge = PathPlanner::Edge;
	using VD = boost::graph_traits<Graph>::vertex_descriptor;
	using ATTR = Track::ATTRIBUTE;

	void spawn_ego_car() override;
	void initialize() override;
    void stop_cars() override;
};
