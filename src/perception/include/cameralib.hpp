#pragma once

#include "LaneDetector.hpp"
#include "SignFastest.hpp"
#include <functional>
#include <image_transport/subscriber.h>
#include <librealsense2/rs.hpp>
#include <ros/node_handle.h>

using namespace std::chrono;

class CameraLib {
  public:
	CameraLib(ros::NodeHandle &nh);
	CameraLib(CameraLib &&) = delete;
	CameraLib(const CameraLib &) = delete;
	CameraLib &operator=(CameraLib &&) = delete;
	CameraLib &operator=(const CameraLib &) = delete;
	~CameraLib();

	std::unique_ptr<tbb::task_group> tasks;

	std::function<void(utils::Lane3 &)> onLaneCompletion;
	std::function<void(utils::Sign &)> onSignCompletion;

  private:
	SignFastest Sign;
	LaneDetector Lane;

	std::thread poll;

	std::atomic<bool> tasks_running{false};

	sensor_msgs::ImagePtr color_msg, depth_msg;

	image_transport::Subscriber rgb_sub;
	image_transport::Subscriber depth_sub;
	image_transport::ImageTransport it;
	cv::Mat depthImage, colorImage;
	cv_bridge::CvImagePtr cv_ptr;
	cv_bridge::CvImagePtr cv_ptr_depth;
	ros::Timer signTimer, laneTimer;

	bool doLane, doSign, realsense, pubImage, useRosTimer, flip, send_depth;
	int mainLoopRate;
	int quality = 30;

	// lock
	std::mutex mutex;

	// rs
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

	void cameraNodeSpin();

	void run_lane_once();
	void run_sign_once();

	void get_frame();

	void depthCallback(const sensor_msgs::ImageConstPtr &msg);
	void imageCallback(const sensor_msgs::ImageConstPtr &msg);
};
