#pragma once

#include "LaneDetector.hpp"
#include "SignFastest.hpp"
#include "TcpClient.hpp"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "ros/ros.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include "yolo-fastestv2.h"
#include <chrono>
#include <librealsense2/rs.hpp>
#include <shared_mutex>
#include <mutex>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <std_msgs/Float32MultiArray.h>
#include <thread>
#include <utility>
#include <vector>
#include <optional>

using namespace std::chrono;

namespace Perception {

inline std::shared_mutex sign_mtx;
inline std::shared_mutex lane_mtx;
inline std::atomic<bool> lane_msg_refreshed{false};
inline std::atomic<bool> sign_msg_refreshed{false};
inline ros::Time last_image_refresh_time;
inline utils::Sign sign_msg;
inline utils::Lane3 lane_msg;
inline void set_lane_msg(const utils::Lane3& m)
{
    std::unique_lock<std::shared_mutex> lock(lane_mtx);
    lane_msg = m;
    lane_msg_refreshed.store(true, std::memory_order_release);
}
inline void set_sign_msg(const utils::Sign& m)
{
    std::unique_lock<std::shared_mutex> lock(sign_mtx);
    sign_msg = m;
    sign_msg_refreshed.store(true, std::memory_order_release);
}
inline std::optional<utils::Lane3> get_lane_msg()
{
    std::shared_lock<std::shared_mutex> lock(lane_mtx);
    if (!lane_msg_refreshed.exchange(false, std::memory_order_acq_rel)) return std::nullopt;
    return lane_msg;
}
inline std::optional<utils::Sign> get_sign_msg()
{
    std::shared_lock<std::shared_mutex> lock(sign_mtx);
    if (!sign_msg_refreshed.exchange(false, std::memory_order_acq_rel)) return std::nullopt;
    return sign_msg;
}

class CameraNode {
  public:
	CameraNode(ros::NodeHandle &nh, bool publish_msg) : it(nh), Sign(nh, publish_msg), Lane(nh, publish_msg) {
		depthImage = cv::Mat::zeros(480, 640, CV_16UC1);
		colorImage = cv::Mat::zeros(480, 640, CV_8UC3);
		std::string nodeName = ros::this_node::getName();
		nh.param(nodeName + "/lane", doLane, true);
		nh.param(nodeName + "/sign", doSign, true);
		nh.param(nodeName + "/realsense", realsense, false);
		nh.param(nodeName + "/rate", mainLoopRate, 50);
		nh.param(nodeName + "/pubImage", pubImage, false);
		nh.param(nodeName + "/flip", flip, false);
		nh.param(nodeName + "/send_depth", send_depth, false);
		nh.param("quality", quality, 30);

		Sign.tcp_client->set_image_quality(quality);

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
			pipe.start(cfg);

			auto profiles = pipe.get_active_profile().get_streams();

			bool found_profile = false;
			for (auto &&p : profiles) {
				if (p.stream_type() == RS2_STREAM_COLOR) {
					auto vid_profile = p.as<rs2::video_stream_profile>();
					rs2_intrinsics intr = vid_profile.get_intrinsics();

					double fx = intr.fx;
					double fy = intr.fy;
					double cx = intr.ppx; // principal point x
					double cy = intr.ppy; // principal point y
					cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

					distCoeffs = (cv::Mat_<double>(1, 5) << intr.coeffs[0], intr.coeffs[1], intr.coeffs[2], intr.coeffs[3], intr.coeffs[4]);
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

		if (doLane) {
			ROS_INFO("starting lane thread");
			lane_thread = std::thread(&CameraNode::run_lane, this);
		}
		if (doSign) {
			ROS_INFO("starting sign thread");
			sign_thread = std::thread(&CameraNode::run_sign, this);
		}
		if (realsense) {
			ROS_INFO("starting realsense thread");
			realsense_thread = std::thread(&CameraNode::run_realsense, this);
		}
		Perception::last_image_refresh_time = ros::Time::now();
	}

	~CameraNode() {
		if (lane_thread.joinable()) {
			lane_thread.join();
		}
		if (sign_thread.joinable()) {
			sign_thread.join();
		}
		if (realsense_thread.joinable()) {
			realsense_thread.join();
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

	bool doLane, doSign, realsense, pubImage, flip, send_depth;
	std::thread lane_thread, sign_thread, realsense_thread;
	int mainLoopRate;
	int quality = 30;

	// lock
	std::mutex frame_mtx;
	std::condition_variable lane_cv;        // wakes the lane‑thread
	std::condition_variable sign_cv;        // wakes the sign‑thread
	bool new_color_for_lane = false;
	bool new_color_for_sign = false;
	bool new_depth_for_sign = false;

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
		cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
		if (cv_ptr_depth == nullptr) {
			ROS_WARN("cv_ptr_depth is null");
			return;
		}
		{
			std::lock_guard<std::mutex> lock(frame_mtx);
			depthImage = cv_ptr_depth->image.clone();
			depthImage.convertTo(depthImage, CV_16UC1);
			if (flip) {
				cv::flip(depthImage, depthImage, -1);
			}
			new_depth_for_sign = true;
		}
		sign_cv.notify_one();
		if (Sign.tcp_client != nullptr && send_depth) {
			Sign.tcp_client->send_image_depth(depthImage);
		}
	}
	void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
		cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
		if (cv_ptr == nullptr) {
			ROS_WARN("cv_ptr is null");
			return;
		}
		{
			std::lock_guard<std::mutex> lock(frame_mtx);
			colorImage = cv_ptr->image.clone();
			if (flip) cv::flip(colorImage, colorImage, -1);
			new_color_for_lane = true;          // lane needs only colour
			new_color_for_sign = true;          // sign needs both; set colour half
			last_image_refresh_time = ros::Time::now();
		}
		lane_cv.notify_one();                   // wake the lane thread
    sign_cv.notify_one();                   // wake the sign thread
		if (Sign.tcp_client != nullptr) {
			if (Lane.showlane) {
				cv::Mat lane_viz = Lane.viz_img;
				if (lane_viz.empty()) {
					ROS_WARN("Lane viz image is empty");
				} else if (lane_viz.size() != cv::Size(640, 480)) {
					cv::resize(lane_viz, lane_viz, cv::Size(640, 480));
				}
				Sign.tcp_client->send_image_rgb(lane_viz);
			} else {
				Sign.tcp_client->send_image_rgb(colorImage);
			}
		}
	}

	void run_lane_once() {
		cv::Mat img;
		{
			std::lock_guard<std::mutex> lock(frame_mtx);
			if (colorImage.empty()) {
				ROS_WARN("colorImage is empty");
				return;
			}
			img = colorImage.clone();
		}
		utils::Lane3 tmp = Lane.publish_lane(img);
		Perception::set_lane_msg(tmp);
	}
	void run_sign_once() {
		cv::Mat color_img, depth_img;
		{
			std::lock_guard<std::mutex> lock(frame_mtx);
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
		utils::Sign tmp = Sign.publish_sign(color_img, depth_img);
		Perception::set_sign_msg(tmp);
	}
	void run_lane()
	{
			if (!doLane) return;

			while (ros::ok())
			{
					std::unique_lock<std::mutex> lk(frame_mtx);
					lane_cv.wait(lk, [&]{ return new_color_for_lane; });

					// Copy the frame while we still hold the mutex
					cv::Mat img = colorImage.clone();
					new_color_for_lane = false;         // we have consumed it
					lk.unlock();                        // release the lock quickly

					utils::Lane3 tmp = Lane.publish_lane(img);
					Perception::set_lane_msg(tmp);
			}
	}
	void run_sign()
	{
			if (!doSign) return;

			while (ros::ok())
			{
					std::unique_lock<std::mutex> lk(frame_mtx);
					sign_cv.wait(lk, [&]{
							return new_color_for_sign && new_depth_for_sign;
					});

					cv::Mat c  = colorImage.clone();
					cv::Mat d  = depthImage.clone();
					new_color_for_sign = false;
					new_depth_for_sign = false;
					lk.unlock();

					utils::Sign tmp = Sign.publish_sign(c, d);
					Perception::set_sign_msg(tmp);
			}
	}
	void run_realsense() {
		static ros::Rate realsense_rate(30);
		while (ros::ok()) {
			get_frame();
			realsense_rate.sleep();
		}
	}
	void get_frame() {
		data = pipe.wait_for_frames();
		auto aligned_frames = align_to_color->process(data);
		color_frame = aligned_frames.get_color_frame();
		depth_frame = aligned_frames.get_depth_frame();
		if (!color_frame || !depth_frame) {
			ROS_WARN("No frame received");
			return;
		}
		{
			std::lock_guard<std::mutex> lock(frame_mtx);
			colorImage = cv::Mat(cv::Size(640, 480), CV_8UC3, (void *)color_frame.get_data(), cv::Mat::AUTO_STEP);
			depthImage = cv::Mat(cv::Size(640, 480), CV_16UC1, (void *)depth_frame.get_data(), cv::Mat::AUTO_STEP);
			if (flip) {
				cv::flip(colorImage, colorImage, -1);
				cv::flip(depthImage, depthImage, -1);
			}
			new_color_for_lane  = true;
			new_color_for_sign  = true;
			new_depth_for_sign  = true;
			last_image_refresh_time = ros::Time::now();
		}
		lane_cv.notify_one();                   // wake the lane thread
		sign_cv.notify_one();                   // wake the sign thread

		if (Sign.tcp_client != nullptr) {
			if (Lane.showlane) {
				cv::Mat lane_viz = Lane.viz_img;
				if (lane_viz.empty()) {
					ROS_WARN("Lane viz image is empty");
				} else if (lane_viz.size() != cv::Size(640, 480)) {
					cv::resize(lane_viz, lane_viz, cv::Size(640, 480));
				}
				Sign.tcp_client->send_image_rgb(lane_viz);
			} else {
				Sign.tcp_client->send_image_rgb(colorImage);
			}
			if (send_depth) {
					Sign.tcp_client->send_image_depth(depthImage);
			}
		}
	}
};

} // namespace Perception