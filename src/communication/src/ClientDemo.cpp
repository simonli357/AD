#include "ClientSubscriber.hpp"
#include "ros/init.h"
#include "ros/node_handle.h"
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

int main(int argc, char *argv[]) {
    ros::init(argc, argv, "client_subscriber_node");
    ros::NodeHandle nh;
    ClientSubscriber client(nh);
    client.init();
    ros::Rate r(100);
    while (ros::ok()) {
        ros::spinOnce();
        r.sleep();
    }
    return 0;
}
