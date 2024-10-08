#include <ros/ros.h>
#include "utility.hpp"

int main(int argc, char **argv) {
    ros::init(argc, argv, "utility_node");
    ros::NodeHandle nh;
    // Utility(ros::NodeHandle& nh_, bool real, double x0, double y0, double yaw0, 
    //  subSign = true,  useEkf = false,  subLane,  robot_name = "car1",  subModel,  subImu = true,  pubOdom = true);
    Utility utility(nh, false, 0,0,0,true, false, false, "car1", true);
    ros::spin();
    return 0;
}
