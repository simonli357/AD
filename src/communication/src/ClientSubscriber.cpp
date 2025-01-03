#include "ClientSubscriber.hpp"
#include "Client.hpp"
#include "ros/node_handle.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32MultiArray.h"

ClientSubscriber::ClientSubscriber(ros::NodeHandle &node_handle) : node_handle(node_handle), client("127.0.0.1", 49153, 10485760) {}

void ClientSubscriber::init() {
	road_object_sub = node_handle.subscribe("/road_objects", 10, &ClientSubscriber::road_object_sub_callback, this);
	camera_sub = node_handle.subscribe("/camera/color/image_raw", 10, &ClientSubscriber::camera_sub_callback, this);
	depth_sub = node_handle.subscribe("/camera/depth/image_raw", 10, &ClientSubscriber::depth_sub_callback, this);
	waypoint_sub = node_handle.subscribe("/waypoints", 10, &ClientSubscriber::waypoint_sub_callback, this);
	sign_sub = node_handle.subscribe("/sign", 10, &ClientSubscriber::sign_sub_callback, this);
	message_sub = node_handle.subscribe("/lane", 10, &ClientSubscriber::message_sub_callback, this);
	client.initialize();
}

void ClientSubscriber::road_object_sub_callback(const std_msgs::Float32MultiArray::ConstPtr &data) { client.send_float32_multi_array(*data); }

void ClientSubscriber::camera_sub_callback(const sensor_msgs::Image::ConstPtr &data) { client.send_image_rgb(*data); }

void ClientSubscriber::depth_sub_callback(const sensor_msgs::Image::ConstPtr &data) { client.send_image_depth(*data); }

void ClientSubscriber::waypoint_sub_callback(const std_msgs::Float32MultiArray::ConstPtr &data) { client.send_float32_multi_array(*data); }

void ClientSubscriber::sign_sub_callback(const std_msgs::Float32MultiArray::ConstPtr &data) { client.send_float32_multi_array(*data); }

void ClientSubscriber::message_sub_callback(const std_msgs::String::ConstPtr &data) { client.send_message(*data); }
