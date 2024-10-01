#include "ros/ros.h"
#include "yolo-fastestv2.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include <chrono>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include "utils/Lane.h"
#include <librealsense2/rs.hpp>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <sensor_msgs/Imu.h>
#include <thread>
#include "LaneDetector.hpp"
#include "SignFastest.hpp"
#include <mutex>


using namespace std::chrono;

class CameraNode {
    public:
        CameraNode(ros::NodeHandle& nh) 
            :it(nh), Sign(nh), Lane(nh)
        {
            // depthImage = cv::Mat::zeros(480, 640, CV_16UC1);
            // colorImage = cv::Mat::zeros(480, 640, CV_8UC3);
            std::string nodeName = ros::this_node::getName();
            nh.param(nodeName + "/lane", doLane, false);
            nh.param(nodeName + "/sign", doSign, false);

            if(Sign.hasDepthImage) {
                std::string topic;
                if (Sign.real) {
                    topic = "/camera/aligned_depth_to_color/image_raw";
                } else {    
                    topic = "/camera/depth/image_raw";
                }
                depth_sub = it.subscribe(topic, 3, &CameraNode::depthCallback, this);
                std::cout << "depth_sub created" << std::endl;
                ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh);
                std::cout << "got it" << std::endl;
            }
            sub = it.subscribe("/camera/color/image_raw", 3, &CameraNode::imageCallback, this);
            std::cout << "waiting for rgb image" << std::endl;
            ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", nh);
            std::cout << "got color image" << std::endl;

            //define rate
            ros::Rate loop_rate(25);

            std::thread t1(&CameraNode::run_lane, this);
            std::thread t2(&CameraNode::run_sign, this);
            while(ros::ok()) {
                ros::spinOnce();
            }
            t1.join();
            t2.join();
        }
        SignFastest Sign;
        LaneDetector Lane;

        sensor_msgs::ImagePtr color_msg, depth_msg;

        image_transport::Subscriber sub;
        image_transport::Subscriber depth_sub;
        image_transport::ImageTransport it;
        cv::Mat depthImage, colorImage;
        cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImagePtr cv_ptr_depth;
        
        bool doLane, doSign;

        //lock
        std::mutex mutex;
       
        void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
            mutex.lock();
            cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            if(cv_ptr_depth == nullptr) {
                std::cout << "cv_ptr_depth is null" << std::endl;
                // mutex.unlock();
                return;
            }
            depthImage = cv_ptr_depth->image;
            mutex.unlock();
        }
        void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
            // mutex.lock();
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if(cv_ptr == nullptr) {
                std::cout << "cv_ptr is null" << std::endl;
                // mutex.unlock();
                return;
            }
            colorImage = cv_ptr->image;
            // if (doLane) {
            //     Lane.publish_lane(cv_ptr->image);
            // }
            // if (doSign) {
            //     Sign.publish_sign(cv_ptr->image, cv_ptr_depth->image);
            // }
            // mutex.unlock();
        }

        void run_lane() {
            static ros::Rate lane_rate(30);
            if (!doLane) {
                ROS_INFO("Lane detection is disabled");
                return;
            }
            while(ros::ok()) {
                if (cv_ptr == nullptr) {
                    std::cout << "cv_ptr is null" << std::endl;
                    lane_rate.sleep();
                    continue;
                }
                if (colorImage.empty()) {
                    std::cout << "colorImage is empty" << std::endl;
                    lane_rate.sleep();
                    continue;
                }
                Lane.publish_lane(colorImage);
                lane_rate.sleep();
            }
        }
        void run_sign() {
            static ros::Rate sign_rate(15);
            if (!doSign) {
                ROS_INFO("Sign detection is disabled");
                return;
            }
            while(ros::ok()) {
                if (cv_ptr == nullptr) {
                    std::cout << "cv_ptr is null" << std::endl;
                    sign_rate.sleep();
                    continue;
                }
                if (cv_ptr_depth == nullptr) {
                    std::cout << "cv_ptr_depth is null" << std::endl;
                    sign_rate.sleep();
                    continue;
                }
                if (colorImage.empty()) {
                    std::cout << "colorImage is empty" << std::endl;
                    sign_rate.sleep();
                    continue;
                }
                if (depthImage.empty()) {
                    std::cout << "depthImage is empty" << std::endl;
                    sign_rate.sleep();
                    continue;
                }
                // Sign.publish_sign(cv_ptr->image, cv_ptr_depth->image);
                Sign.publish_sign(colorImage, depthImage);
                sign_rate.sleep();
            }
        }
};

int main(int argc, char **argv) {
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    int opt;

    // Initialize ROS node and publisher
    ros::init(argc, argv, "object_detector2");
    ros::NodeHandle nh;

    CameraNode CameraNode(nh);

    return 0;
}