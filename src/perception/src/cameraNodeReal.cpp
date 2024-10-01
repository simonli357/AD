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
            :Sign(nh), Lane(nh)//, align_to_color(RS2_STREAM_COLOR)
        {

            std::string nodeName = ros::this_node::getName();
            nh.param(nodeName + "/lane", doLane, false);
            nh.param(nodeName + "/sign", doSign, false);

            depthImage = cv::Mat::zeros(480, 640, CV_16UC1);
            colorImage = cv::Mat::zeros(480, 640, CV_8UC3);

            // rs
            align_to_color = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
            depth_frame = rs2::frame();
            color_frame = rs2::frame();
            imu_msg = sensor_msgs::Imu();
            data = rs2::frameset();

            cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
            cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
            cfg.enable_stream(RS2_STREAM_GYRO);
            cfg.enable_stream(RS2_STREAM_ACCEL);
            pipe.start(cfg);

            std::cout.precision(4);
            imu_pub = nh.advertise<sensor_msgs::Imu>("/camera/imu", 2);
            color_pub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1);
            depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_rect_raw", 1);
            std::cout <<"pub created" << std::endl; 
        }

        SignFastest Sign;
        LaneDetector Lane;

    // private:
        
        bool doLane, doSign;

        cv::Mat colorImage;
        cv::Mat depthImage;
        
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

        void get_frames() {
            while (true) {
                data = pipe.wait_for_frames();
                auto aligned_frames = align_to_color->process(data);
                color_frame = aligned_frames.get_color_frame();
                depth_frame = aligned_frames.get_depth_frame();
                gyro_frame = data.first_or_default(RS2_STREAM_GYRO);
                accel_frame = data.first_or_default(RS2_STREAM_ACCEL);
                if (!color_frame || !depth_frame) {
                    ROS_INFO("No frame received");
                    continue;
                }
                colorImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void*)depth_frame.get_data(), cv::Mat::AUTO_STEP);

                // Convert gyro and accel frames to ROS Imu message
                imu_msg.header.stamp = ros::Time::now();
                imu_msg.header.frame_id = "imu_link";
                float *gyro_data = (float *)gyro_frame.get_data();
                imu_msg.angular_velocity.x = gyro_data[0];
                imu_msg.angular_velocity.y = gyro_data[1];
                imu_msg.angular_velocity.z = gyro_data[2];

                float *accel_data = (float *)accel_frame.get_data();
                imu_msg.linear_acceleration.x = accel_data[0];
                imu_msg.linear_acceleration.y = accel_data[1];
                imu_msg.linear_acceleration.z = accel_data[2];
                imu_pub.publish(imu_msg);

                // ROS_INFO("frame received");
                // Convert color Mat to ROS Image message
                // color_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", colorImage).toImageMsg();

                // // Convert depth Mat to ROS Image message
                // depth_msg = cv_bridge::CvImage(std_msgs::Header(), "mono16", depthImage).toImageMsg();

                // // Publish color, depth, and imu data
                // color_pub.publish(color_msg);
                // depth_pub.publish(depth_msg);
            }
        }

        void run_lane() {
            static ros::Rate lane_rate(30);
            while(ros::ok()) {
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
            while(ros::ok()) {
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
    //define rate
    ros::Rate loop_rate(25);

    std::thread t1(&CameraNode::run_lane, &CameraNode);
    std::thread t2(&CameraNode::get_frames, &CameraNode);
    // Spin ROS node
    while(ros::ok()) {
        CameraNode.run_sign();
    }
    t1.join();
    t2.join();

    return 0;
}