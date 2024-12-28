#include "opencv2/videoio.hpp"
#include <Client.hpp>
#include <chrono>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <unistd.h>
#include <opencv2/opencv.hpp>
#include <cv_bridge/cv_bridge.h>

sensor_msgs::Image create_image(const std::string& image_file) {
    // Load the image using OpenCV (BGR format by default)
    cv::Mat cv_image = cv::imread(image_file, cv::IMREAD_COLOR);  // Read the image as BGR

    if (cv_image.empty()) {
        throw std::runtime_error("Failed to load image");
    }

    // Use CvBridge to convert cv::Mat to sensor_msgs::Image
    cv_bridge::CvImage img_bridge;
    img_bridge.header.frame_id = "camera_frame"; // Set frame id
    img_bridge.encoding = sensor_msgs::image_encodings::BGR8; // Set encoding
    img_bridge.image = cv_image;  // Set the cv::Mat image

    // Convert to sensor_msgs::Image message
    sensor_msgs::Image img_msg;
    img_bridge.toImageMsg(img_msg);  // Convert to ROS Image message

    return img_msg;
}

sensor_msgs::Image image_from_frame(cv::Mat& frame) {
    cv_bridge::CvImage img_bridge;
    img_bridge.header.frame_id = "camera_frame"; // Set frame id
    img_bridge.encoding = sensor_msgs::image_encodings::BGR8; // Set encoding
    img_bridge.image = frame;  // Set the cv::Mat image
    // Convert to sensor_msgs::Image message
    sensor_msgs::Image img_msg;
    img_bridge.toImageMsg(img_msg);  // Convert to ROS Image message
    return img_msg;
}

int main(int argc, char *argv[]) {
	Client client("127.0.0.1", 49153, 10485760);
	client.initialize();
    cv::VideoCapture cap(0);
    if(!cap.isOpened()) {
        std::cerr << "Error: Could not open camera." << std::endl;
    }
    cv::Mat frame;
	while (true) {
        cap >> frame;
        client.send_image(image_from_frame(frame));
		if (!client.get_strings().empty()) {
			std::cout << client.get_strings().front() << std::endl;
			client.get_strings().pop();
		}
        std::this_thread::sleep_for(std::chrono::milliseconds(50));
	}
	return 0;
}
