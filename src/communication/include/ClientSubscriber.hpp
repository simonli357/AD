#include "Client.hpp"
#include "ros/node_handle.h"
#include "ros/subscriber.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"

class ClientSubscriber {
  public:
	ClientSubscriber(ros::NodeHandle &node_handle);
	ClientSubscriber(ClientSubscriber &&) = default;
	ClientSubscriber(const ClientSubscriber &) = delete;
	ClientSubscriber &operator=(ClientSubscriber &&) = delete;
	ClientSubscriber &operator=(const ClientSubscriber &) = delete;
	~ClientSubscriber() = default;

	void init();

  private:
	Client client;
	ros::NodeHandle &node_handle;

	ros::Subscriber road_object_sub;
	ros::Subscriber camera_sub;
	ros::Subscriber depth_sub;
	ros::Subscriber waypoint_sub;
	ros::Subscriber sign_sub;
	ros::Subscriber message_sub;

	void road_object_sub_callback(const std_msgs::Float32MultiArray::ConstPtr &data);
	void camera_sub_callback(const sensor_msgs::Image::ConstPtr &data);
	void depth_sub_callback(const sensor_msgs::Image::ConstPtr &data);
	void waypoint_sub_callback(const std_msgs::Float32MultiArray::ConstPtr &data);
	void sign_sub_callback(const std_msgs::Float32MultiArray::ConstPtr &data);
	void message_sub_callback(const std_msgs::String::ConstPtr &data);
};
