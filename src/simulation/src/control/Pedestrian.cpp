#include "Pedestrian.hpp"
#include <geometry_msgs/PoseStamped.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <random>
#include <cmath>

Pedestrian::Pedestrian(TrafficManager &traffic_manager, ros::NodeHandle &nh, std::string name) : traffic_manager(traffic_manager), nh(nh) {
	this->pedestrian_name = name;
	setup();
	main = std::thread(&Pedestrian::run, this);
}

Pedestrian::~Pedestrian() {
	if (main.joinable()) {
		main.join();
	}
}

void Pedestrian::run() {
	double dist_from_car = 2.0;
	std::mt19937 rng(std::random_device{}());
	std::uniform_real_distribution<> delay_dist(10.0, 20.0);
	while (ros::ok() && alive) {
		double ego_x;
		double ego_y;
		traffic_manager.get_car_position("car1", ego_x, ego_y);
		Vertex u = traffic_manager.planner->track.find_closest_node(ego_x, ego_y);
		Vertex v = traffic_manager.planner->track.find_first_neighbor(u);
		if (u.id == v.id) {
			// No neighbors
			ros::Duration(0.1).sleep();
			continue;
		}
		// Spawn pedestrian in front of car
		double dx = v.x - u.x;
		double dy = v.y - u.y;
		double yaw = atan2(dy, dx);
		// Calculate pedestrian position.
		double pedestrian_x = u.x + dist_from_car * std::cos(yaw);
		double pedestrian_y = u.y + dist_from_car * std::sin(yaw);
        Vertex p = traffic_manager.planner->track.find_closest_node(pedestrian_x, pedestrian_y);
		move_pedestrian_to(p.x, p.y, yaw);
		ros::Duration(8.0).sleep();
		hide_pedestrian();
		double delay = delay_dist(rng);
		ros::Duration(delay).sleep();
	}
}

void Pedestrian::setup() { teleport_pub = nh.advertise<geometry_msgs::PoseStamped>("/" + pedestrian_name + "/localisation/teleport", 1); }

void Pedestrian::stop() { alive = false; }

void Pedestrian::move_pedestrian_to(double x, double y, double yaw) {
	geometry_msgs::PoseStamped cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "world";
	cmd.pose.position.x = x;
	cmd.pose.position.y = y;
	cmd.pose.position.z = gazebo_z;
	tf2::Quaternion q;
	q.setRPY(0, 0, yaw);
	cmd.pose.orientation = tf2::toMsg(q);
	teleport_pub.publish(cmd);
}

void Pedestrian::hide_pedestrian() {
	geometry_msgs::PoseStamped cmd;
	cmd.header.stamp = ros::Time::now();
	cmd.header.frame_id = "world";
	cmd.pose.position.x = 0;
	cmd.pose.position.y = 0;
	cmd.pose.position.z = -10;
	tf2::Quaternion q;
	q.setRPY(0, 0, 0);
	cmd.pose.orientation = tf2::toMsg(q);
	teleport_pub.publish(cmd);
}
