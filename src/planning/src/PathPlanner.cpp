#include <ros/node_handle.h>

int main(int argc, char **argv) {
	ros::init(argc, argv, "planner", ros::init_options::NoSigintHandler | ros::init_options::AnonymousName);
	ros::NodeHandle nh;
    std::cout << "TEST NODE - Planning" << std::endl;
	return 0;
}
