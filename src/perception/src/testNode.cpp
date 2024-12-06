#include "ros/ros.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "LaneDetector.hpp"
#include "SignFastest.hpp"
#include <mutex>
#include <opencv2/opencv.hpp>

class TestNode {
public:
    TestNode(ros::NodeHandle& nh)
        : it(nh), Sign(nh), Lane(nh) {
        std::string nodeName = ros::this_node::getName();
        nh.param(nodeName + "/lane", doLane, true);
        nh.param(nodeName + "/sign", doSign, true);
        nh.param(nodeName + "/rate", mainLoopRate, 30);
        nh.param(nodeName + "/input_type", inputType, std::string("image")); // "image" or "video"
        nh.param(nodeName + "/input_path", inputPath, std::string("./rf2/381.jpg"));

        std::string path = getSourceDirectory();
        //check if input path starts with /
        if (inputPath[0] != '/') {
            path += "/";
        }
        path += inputPath;
        if (inputType == "video") {
            videoCapture.open(path);
            if (!videoCapture.isOpened()) {
                ROS_ERROR("Failed to open video file: %s", path.c_str());
                ros::shutdown();
                return;
            }
        } else if (inputType == "image") {
            colorImage = cv::imread(path, cv::IMREAD_COLOR);
            if (colorImage.empty()) {
                ROS_ERROR("Failed to load image file: %s", path.c_str());
                ros::shutdown();
                return;
            }
        } else {
            ROS_ERROR("Unsupported input type: %s", inputType.c_str());
            ros::shutdown();
            return;
        }

        ros::Rate loopRate(mainLoopRate);
        while (ros::ok()) {
            if (inputType == "video") {
                processVideoFrame();
            } else if (inputType == "image") {
                processImageFrame();
            }
            ros::spinOnce();
            loopRate.sleep();
        }
    }

private:
    image_transport::ImageTransport it;
    SignFastest Sign;
    LaneDetector Lane;

    cv::Mat colorImage;
    cv::VideoCapture videoCapture;

    bool doLane, doSign;
    int mainLoopRate;
    std::string inputType; // "image" or "video"
    std::string inputPath; // Path to image or video file

    void processVideoFrame() {
        if (!videoCapture.read(colorImage)) {
            ROS_INFO("End of video reached.");
            ros::shutdown();
            return;
        }
        if (colorImage.empty()) {
            ROS_WARN("Captured empty frame from video.");
            return;
        }
        runDetection();
    }

    std::string getSourceDirectory() {
        std::string file_path(__FILE__);  // __FILE__ is the full path of the source file
        size_t last_dir_sep = file_path.rfind('/');  // For Unix/Linux path
        if (last_dir_sep == std::string::npos) {
            last_dir_sep = file_path.rfind('\\');  // For Windows path
        }
        if (last_dir_sep != std::string::npos) {
            return file_path.substr(0, last_dir_sep);  // Extract directory path
        }
        return "";  // Return empty string if path not found
    }
    void processImageFrame() {
        if (colorImage.empty()) {
            ROS_WARN("Loaded empty image.");
            return;
        }
        runDetection();
    }

    void runDetection() {
        if (doLane) {
            Lane.publish_lane(colorImage);
        }
        if (doSign) {
            // Dummy depth image since we are processing a static image or video
            cv::Mat dummyDepth = cv::Mat::zeros(colorImage.size(), CV_16UC1);
            Sign.publish_sign(colorImage, dummyDepth);
        }
    }
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "object_detector_image_video");
    ros::NodeHandle nh;

    TestNode cameraNode(nh);

    return 0;
}
