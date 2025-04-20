#include "TcpClient.hpp"
#include "LaneDetector.hpp"
#include "SignFastest.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "yolo-fastestv2.h"
#include <chrono>
#include <librealsense2/rs.hpp>
#include <mutex>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <thread>
#include <vector>

using namespace std::chrono;

class CameraNode {
  public:
	CameraNode(ros::NodeHandle &nh) : it(nh), Sign(nh), Lane(nh) {
		depthImage = cv::Mat::zeros(480, 640, CV_16UC1);
		colorImage = cv::Mat::zeros(480, 640, CV_8UC3);
		std::string nodeName = ros::this_node::getName();
		nh.param(nodeName + "/lane", doLane, true);
		nh.param(nodeName + "/sign", doSign, true);
		nh.param(nodeName + "/realsense", realsense, false);
		nh.param(nodeName + "/rate", mainLoopRate, 50);
		nh.param(nodeName + "/pubImage", pubImage, false);
		nh.param(nodeName + "/thread", useRosTimer, false);
		nh.param(nodeName + "/flip", flip, false);
		nh.param(nodeName + "/send_depth", send_depth, false);
		nh.param("quality", quality, 30);

		if (!realsense) {
			if (Sign.hasDepthImage) {
				std::string topic;
				bool is_real;
				if (!nh.getParam(nodeName + "/real", is_real)) {
					ROS_WARN("Failed to get 'real' parameter. Defaulting to false.");
				}
				if (is_real) {
					std::cout << "real, depth topic is /camera/aligned_depth_to_color/image_raw" << std::endl;
					topic = "/camera/aligned_depth_to_color/image_raw";
				} else {
					std::cout << "not real, depth topic is /camera/depth/image_raw" << std::endl;
					topic = "/camera/depth/image_raw";
				}
				depth_sub = it.subscribe(topic, 3, &CameraNode::depthCallback, this);
				std::cout << "depth_sub created, waiting for " << topic << std::endl;
				ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh);
				std::cout << "got it" << std::endl;
			}
			rgb_sub = it.subscribe("/camera/color/image_raw", 3, &CameraNode::imageCallback, this);
			std::cout << "waiting for rgb image" << std::endl;
			ros::topic::waitForMessage<sensor_msgs::Image>("/camera/color/image_raw", nh);
			std::cout << "got color image" << std::endl;
		} else {
			align_to_color = std::make_unique<rs2::align>(RS2_STREAM_COLOR);
			depth_frame = rs2::frame();
			color_frame = rs2::frame();
			data = rs2::frameset();

			cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
			cfg.enable_stream(RS2_STREAM_DEPTH, 640, 480, RS2_FORMAT_Z16, 30);
			cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
			cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
			pipe.start(cfg);

			auto profiles = pipe.get_active_profile().get_streams();

			bool found_profile = false;
			for (auto &&p : profiles)
			{
					if (p.stream_type() == RS2_STREAM_COLOR)
					{
							auto vid_profile = p.as<rs2::video_stream_profile>();
							rs2_intrinsics intr = vid_profile.get_intrinsics();

							double fx = intr.fx;
							double fy = intr.fy;
							double cx = intr.ppx; // principal point x
							double cy = intr.ppy; // principal point y
							cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 
																												0, fy, cy, 
																												0, 0, 1);

							distCoeffs = (cv::Mat_<double>(1,5) << intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]);
							cv::initUndistortRectifyMap(cameraMatrix, distCoeffs, cv::Mat(), cameraMatrix, cv::Size(640, 480), CV_16SC2, map1, map2);
							ROS_INFO("camera intrinsics: fx=%.2f, fy=%.2f, cx=%.2f, cy=%.2f", fx, fy, cx, cy);
							ROS_INFO("distortion coefficients: %.2f, %.2f, %.2f, %.2f, %.2f", intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]);
						  found_profile = true;
							break;
					}
			}

			if (!found_profile) {
				ROS_ERROR("FATAL ERROR: No color profile found");
				exit(1);
			}

			std::cout.precision(4);
			if (pubImage) {
				color_pub = nh.advertise<sensor_msgs::Image>("/camera/color/image_raw", 1);
				depth_pub = nh.advertise<sensor_msgs::Image>("/camera/depth/image_raw", 1);
				std::cout << "pub created" << std::endl;
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
				// laneTimer = nh.createTimer(ros::Duration(1.0 / mainLoopRate), &CameraNode::lane_timer_callback, this);
				lane_thread = std::thread(&CameraNode::run_lane, this);
			}
			if (doSign) {
				ROS_INFO("starting sign timer");
				// signTimer = nh.createTimer(ros::Duration(1.0 / mainLoopRate), &CameraNode::sign_timer_callback, this);
				sign_thread = std::thread(&CameraNode::run_sign, this);
			}
		}
	}

	~CameraNode() {
			if (lane_thread.joinable()) {
					lane_thread.join();
			}
			if (sign_thread.joinable()) {
					sign_thread.join();
			}
	}

	void cameraNodeSpin() {
			if (realsense) {
				ros::Rate cameraRate(30);
				while (ros::ok()) {
					get_frame();
					cameraRate.sleep();
				}
			} else {
				ros::Rate loopRate(mainLoopRate);
				while (ros::ok()) {
						ros::spinOnce();
						if (!realsense && !useRosTimer) {
								// If not using realsense or timers, 
								// detection might happen in imageCallback.
						}
						loopRate.sleep();
				}
			}
	}

	SignFastest Sign;
	LaneDetector Lane;
    
	sensor_msgs::ImagePtr color_msg, depth_msg;

	image_transport::Subscriber rgb_sub;
	image_transport::Subscriber depth_sub;
	image_transport::ImageTransport it;
	cv::Mat depthImage, colorImage;
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_depth;
	ros::Timer signTimer, laneTimer;

	bool doLane, doSign, realsense, pubImage, useRosTimer, flip, send_depth;
	std::thread lane_thread, sign_thread;
	int mainLoopRate;
	int quality = 30;

	// lock
	std::mutex mutex;

	// rs
	ros::Publisher color_pub, depth_pub;

	rs2::pipeline pipe;
	rs2::config cfg;
	rs2::frame color_frame;
	rs2::frame depth_frame;
	rs2::frameset data;
	rs2::frame gyro_frame;
	rs2::frame accel_frame;
	std::unique_ptr<rs2::align> align_to_color;
	cv::Mat cameraMatrix;
	cv::Mat distCoeffs;
	cv::Mat map1, map2;

	void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
		// mutex.lock();
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		if (cv_ptr_depth == nullptr) {
			ROS_WARN("cv_ptr_depth is null");
			// mutex.unlock();
			return;
		}
		{
			std::lock_guard<std::mutex> lock(mutex);
			depthImage = cv_ptr_depth->image.clone();
			if (flip) {
				cv::flip(depthImage, depthImage, -1);
			}
		}
		if (Sign.tcp_client != nullptr && send_depth) {
        	Sign.tcp_client->send_image_depth(depthImage);
		}
		// mutex.unlock();
	}
	void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
		// mutex.lock();
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		if (cv_ptr == nullptr) {
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
			std::lock_guard<std::mutex> lock(mutex);
      colorImage = cv_ptr->image.clone();
			if (flip) cv::flip(colorImage, colorImage, -1);
		}
		if (Sign.tcp_client != nullptr) {
        	Sign.tcp_client->send_image_rgb(colorImage, quality);
		}
		// mutex.unlock();
	}

	void lane_timer_callback(const ros::TimerEvent &event) { run_lane_once(); }
	void sign_timer_callback(const ros::TimerEvent &event) { run_sign_once(); }
	void run_lane_once() {
		cv::Mat img;
		{
				std::lock_guard<std::mutex> lock(mutex);
				if (colorImage.empty()) {
						ROS_WARN("colorImage is empty");
						return;
				}
				img = colorImage.clone();
		}
		Lane.publish_lane(img);
	}
	void run_sign_once() {
		cv::Mat color_img, depth_img;
		{
				std::lock_guard<std::mutex> lock(mutex);
				if (colorImage.empty()) {
						ROS_WARN("colorImage is empty");
						return;
				}
				if (depthImage.empty()) {
						ROS_WARN("depthImage is empty");
						return;
				}
				color_img = colorImage.clone();
				depth_img = depthImage.clone();
		}
		Sign.publish_sign(color_img, depth_img);
	}
	void run_lane() {
		static ros::Rate lane_rate(30);
		if (!doLane) {
			return;
		}
		while (ros::ok()) {
			run_lane_once();
			lane_rate.sleep();
		}
	}
	void run_sign() {
		static ros::Rate sign_rate(30);
		if (!doSign) {
			return;
		}
		while (ros::ok()) {
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
		colorImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
		depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
		if (flip) {
			cv::flip(colorImage, colorImage, -1);
			cv::flip(depthImage, depthImage, -1);
		}
		// cv::remap(colorImage, colorImage, map1, map2, cv::INTER_LINEAR);
		// cv::remap(depthImage, depthImage, map1, map2, cv::INTER_NEAREST);

		if (!useRosTimer) {
			if (doLane) {
				run_lane_once();
			}
			if (doSign) {
				run_sign_once();
			}
		}
		if (Sign.tcp_client != nullptr) {
			Sign.tcp_client->send_image_rgb(colorImage);
			// Sign.tcp_client->send_image_depth(depthImage);
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

	CameraNode cameraNode(nh);
	cameraNode.cameraNodeSpin();

	return 0;
}
