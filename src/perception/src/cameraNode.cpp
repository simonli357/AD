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
#include <librealsense2/rs.hpp>

using namespace std::chrono;

class CameraNode {
    public:
        CameraNode(ros::NodeHandle& nh) 
            :it(nh), Sign(nh), Lane(nh)
        {
            depthImage = cv::Mat::zeros(480, 640, CV_16UC1);
            colorImage = cv::Mat::zeros(480, 640, CV_8UC3);
            std::string nodeName = ros::this_node::getName();
            nh.param(nodeName + "/lane", doLane, true);
            nh.param(nodeName + "/sign", doSign, true);
            nh.param(nodeName + "/realsense", realsense, false);
            nh.param(nodeName + "/rate", mainLoopRate, 50);
            nh.param(nodeName + "/pubImage", pubImage, false);
            nh.param(nodeName + "/thread", useRosTimer, false);

            if(!realsense) {
                if(Sign.hasDepthImage) {
                    std::string topic;
                    if (Sign.real) {
                        topic = "/camera/aligned_depth_to_color/image_raw";
                    } else {    
                        topic = "/camera/depth/image_raw";
                    }
                    depth_sub = it.subscribe(topic, 3, &CameraNode::depthCallback, this);
                    std::cout << "depth_sub created, waiting for " << topic << std::endl;
                    ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh);
                    std::cout << "got it" << std::endl;
                }
                sub = it.subscribe("/camera/color/image_raw", 3, &CameraNode::imageCallback, this);
                std::cout << "waiting for rgb image" << std::endl;
                ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", nh);
                std::cout << "got color image" << std::endl;
            } else {
                align_to_color = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
                depth_frame = rs2::frame();
                color_frame = rs2::frame();
                imu_msg = sensor_msgs::Imu();
                imu_msg.header.frame_id = "imu_link";
                data = rs2::frameset();

                cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
                cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
                cfg.enable_stream(RS2_STREAM_GYRO);
                cfg.enable_stream(RS2_STREAM_ACCEL);
                pipe.start(cfg);

                std::cout.precision(4);
                imu_pub = nh.advertise<sensor_msgs::Imu>("/camera/imu", 2);
                if (pubImage) {
                    color_pub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1);
                    depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1);
                    std::cout <<"pub created" << std::endl; 
                }
            }

            if (!doLane) {
                ROS_WARN("Lane detection is disabled");
            }
            if (!doSign) {
                ROS_WARN("Sign detection is disabled");
            }
            if (useRosTimer) {
                ROS_INFO("RosTimer is enabled");
            } else {
                ROS_INFO("RosTimer is disabled");
            }

            if (useRosTimer) {
                if (doLane) {
                    ROS_INFO("starting lane timer");
                    laneTimer = nh.createTimer(ros::Duration(1.0 / mainLoopRate), &CameraNode::lane_timer_callback, this);
                }
                if (doSign) {
                    ROS_INFO("starting sign timer");
                    signTimer = nh.createTimer(ros::Duration(1.0 / mainLoopRate), &CameraNode::sign_timer_callback, this);
                }
                // if (doLane) {
                //     std::thread t1(&CameraNode::run_lane, this);
                //     t1.detach();
                // }
                // if (doSign) {
                //     std::thread t2(&CameraNode::run_sign, this);
                //     t2.detach();
                // }
            }
            ros::Rate loopRate(this->mainLoopRate);
            while(ros::ok()) {
                ros::spinOnce();
                if(realsense) {
                    get_frame();
                }
                loopRate.sleep();
            }
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
        ros::Timer signTimer, laneTimer;
        
        bool doLane, doSign, realsense, pubImage, useRosTimer;
        int mainLoopRate;

        //lock
        std::mutex mutex;

        // rs
        ros::Publisher imu_pub;
        ros::Publisher color_pub, depth_pub;

        rs2::pipeline pipe;
        rs2::config cfg;
        rs2::frame color_frame;
        rs2::frame depth_frame;
        sensor_msgs::Imu imu_msg;
        rs2::frameset data;
        rs2::frame gyro_frame;
        rs2::frame accel_frame;
        std::unique_ptr<rs2::align> align_to_color;
       
        void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
            // mutex.lock();
            cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            if(cv_ptr_depth == nullptr) {
                ROS_WARN("cv_ptr_depth is null");
                // mutex.unlock();
                return;
            }
            depthImage = cv_ptr_depth->image;
            // mutex.unlock();
        }
        void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
            // mutex.lock();
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if(cv_ptr == nullptr) {
                ROS_WARN("cv_ptr is null");
                // mutex.unlock();
                return;
            }
            if (!useRosTimer) {
                if (doLane) {
                    Lane.publish_lane(cv_ptr->image);
                }
                if (doSign) {
                    Sign.publish_sign(cv_ptr->image, cv_ptr_depth->image);
                }
            } else {
                colorImage = cv_ptr->image;
            }
            // mutex.unlock();
        }

        void lane_timer_callback(const ros::TimerEvent& event) {
            run_lane_once();
        }
        void sign_timer_callback(const ros::TimerEvent& event) {
            run_sign_once();
        }
        void run_lane_once() {
            if (colorImage.empty()) {
                ROS_WARN("colorImage is empty");
                return;
            }
            Lane.publish_lane(colorImage);
        }
        void run_sign_once() {
            if (colorImage.empty()) {
                ROS_WARN("colorImage is empty");
                return;
            }
            if (depthImage.empty()) {
                ROS_WARN("depthImage is empty");
                return;
            }
            Sign.publish_sign(colorImage, depthImage);
        }
        void run_lane() {
            static ros::Rate lane_rate(50);
            if (!doLane) {
                return;
            }
            while(ros::ok()) {
                run_lane_once();
                lane_rate.sleep();
            }
        }
        void run_sign() {
            static ros::Rate sign_rate(50);
            if (!doSign) {
                return;
            }
            while(ros::ok()) {
                run_sign_once();
                sign_rate.sleep();
            }
        }
        void get_frame() {
            data = pipe.wait_for_frames();
            auto aligned_frames = align_to_color->process(data);
            color_frame = aligned_frames.get_color_frame();
            depth_frame = aligned_frames.get_depth_frame();
            gyro_frame = data.first_or_default(RS2_STREAM_GYRO);
            accel_frame = data.first_or_default(RS2_STREAM_ACCEL);
            if (!color_frame || !depth_frame) {
                ROS_WARN("No frame received");
                return;
            }
            colorImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
            depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

            // Convert gyro and accel frames to ROS Imu message
            imu_msg.header.stamp = ros::Time::now();
            float *gyro_data = (float *)gyro_frame.get_data();
            imu_msg.angular_velocity.x = gyro_data[0];
            imu_msg.angular_velocity.y = gyro_data[1];
            imu_msg.angular_velocity.z = gyro_data[2];

            float *accel_data = (float *)accel_frame.get_data();
            imu_msg.linear_acceleration.x = accel_data[0];
            imu_msg.linear_acceleration.y = accel_data[1];
            imu_msg.linear_acceleration.z = accel_data[2];
            imu_pub.publish(imu_msg);

            if (!useRosTimer) {
                if (doLane) {
                    run_lane_once();
                }
                if (doSign) {
                    run_sign_once();
                }
            }
            if (pubImage) {
                color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImage).toImageMsg();
                depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthImage).toImageMsg();
                color_pub.publish(color_msg);
                depth_pub.publish(depth_msg);
            }
        }
};

int main(int argc, char **argv) {
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;

    ros::init(argc, argv, "object_detector2");
    ros::NodeHandle nh;

    CameraNode CameraNode(nh);

    return 0;
}