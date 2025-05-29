#include "Perception.hpp"

int main(int argc, char **argv) {
	std::cout << "OpenCV version : " << CV_VERSION << std::endl;

	ros::init(argc, argv, "object_detector2");
	ros::NodeHandle nh;

	Perception::CameraNode cameraNode(nh, true);
	if (!cameraNode.realsense) {
		ros::Rate loopRate(cameraNode.mainLoopRate);
		while (ros::ok()) {
			ros::spinOnce();
			loopRate.sleep();
		}
	} else {
		ros::waitForShutdown(); 
	}

	return 0;
}
