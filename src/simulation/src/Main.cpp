#include <ros/init.h>
#include <ros/node_handle.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "traffic_node");
	ros::NodeHandle nh;
    ros::spin();
	return 0;
}
