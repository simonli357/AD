#pragma once

#include "OldLaneDetector.hpp"
#include "stdafx.h"
#include "utils/Lane2.h"
#include "std_msgs/Float32.h"
#include "utils/constants.h"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>
#include "utils/constants.h"

using namespace std::chrono;
using namespace Eigen;

class IPMCamera {
	public:
			IPMCamera(ros::NodeHandle& nh, bool useNearest = true)
				: _useNearest(useNearest)
			{

					using namespace VehicleConstants;
					// 1) build intrinsic K
					double fx = CAMERA_PARAMS_REAL[0], fy = CAMERA_PARAMS_REAL[1],
								 cx = CAMERA_PARAMS_REAL[2], cy = CAMERA_PARAMS_REAL[3];
					cv::Mat K = (cv::Mat_<double>(3, 3) <<
							fx,  0, cx,
							 0, fy, cy,
							 0,  0,  1);
	
					// 2) build rotation R from yaw/pitch/roll (in radians)
					double yaw   = REALSENSE_TF_REAL[5];
					double pitch = REALSENSE_TF_REAL[4];
					double roll  = REALSENSE_TF_REAL[3];
	
					cv::Mat Rz = (cv::Mat_<double>(3,3) <<
							std::cos(-yaw), -std::sin(-yaw), 0,
							std::sin(-yaw),  std::cos(-yaw), 0,
							0,               0,              1);
	
					cv::Mat Ry = (cv::Mat_<double>(3,3) <<
							std::cos(-pitch), 0, std::sin(-pitch),
							0,                1, 0,
						 -std::sin(-pitch), 0, std::cos(-pitch));
	
					cv::Mat Rx = (cv::Mat_<double>(3,3) <<
							1, 0,               0,
							0, std::cos(-roll), -std::sin(-roll),
							0, std::sin(-roll),  std::cos(-roll));
	
					// axis switch matrix (x = –y, y = –z, z = x)
					cv::Mat Rs = (cv::Mat_<double>(3,3) <<
							0, -1,  0,
							0,  0, -1,
							1,  0,  0);
	
					cv::Mat R = Rs * (Rz * (Ry * Rx));
	
					// 3) build translation t = –R * C
					cv::Mat C = (cv::Mat_<double>(3,1) << REALSENSE_TF_REAL[0],
							REALSENSE_TF_REAL[1],
							REALSENSE_TF_REAL[2]);
					// cv::Mat C = (cv::Mat_<double>(3,1) << 0,
					// 		0,
					// 		REALSENSE_TF_REAL[2]);
					cv::Mat t = -R * C;
	
					// 4) form [R|t] and projection P = K * [R|t]
					cv::Mat Rt(3,4,CV_64F);
					R.copyTo(Rt(cv::Rect(0,0,3,3)));
					t.copyTo(Rt(cv::Rect(3,0,1,3)));
					cv::Mat P = K * Rt;
	
					// 5) build your M matrix (4×3)
					if (!nh.getParam("/resolution", resolution)) {
							ROS_ERROR("Failed to get param '/resolution'");
							exit(1);
					}
					double pxPerM = double(resolution);
					if (!nh.getParam("/far_m", far_m)) {
							ROS_ERROR("Failed to get param '/far_m'");
							exit(1);
					}
					if (!nh.getParam("/near_m", near_m)) {
							ROS_ERROR("Failed to get param '/near_m'");
							exit(1);
					}
					if (!nh.getParam("/width_m", width_m)) {
							ROS_ERROR("Failed to get param '/width_m'");
							exit(1);
					}
					double CONSTANT_SHIFT = 0.585;
					cv::Mat M = (cv::Mat_<double>(4,3) <<
							 1.0/pxPerM,         0.0, near_m,
							 0.0,         -1.0/pxPerM,  width_m/2.0,
							 0.0,                0.0,           0.0,
							 0.0,                0.0,           1.0);
	
					// 6) compute the 3×3 homography H = P * M, then invert for IPM
					cv::Mat H = P * M;
					_ipmTransform = H.inv();
	
					// 7) store desired output size
					double bev_length_m = far_m - near_m;
					_outputSize = cv::Size(
							int((far_m-near_m) * pxPerM),   // now this is the horizontal span
							int(width_m        * pxPerM)    // and this is the vertical span
					);
					_mapX.create(_outputSize, CV_32FC1);
					_mapY.create(_outputSize, CV_32FC1);
					
					cv::Mat Hf = H;
					for(int v = 0; v < _outputSize.height; ++v) {
						for(int u = 0; u < _outputSize.width; ++u) {
							// build a 3×1 double vector
							cv::Mat dst = (cv::Mat_<double>(3,1) << double(u),
																											double(v),
																											1.0);
							// forward map
							cv::Mat src = Hf * dst;
							double w = src.at<double>(2,0);
							// now divide—w should be nonzero
							_mapX.at<float>(v,u) = float(src.at<double>(0,0) / w);
							_mapY.at<float>(v,u) = float(src.at<double>(1,0) / w);
						}
					}
			}
	
			bool getIPM(const cv::Mat& in, cv::Mat& out) const {
					int interp = _useNearest ? cv::INTER_NEAREST : cv::INTER_LINEAR;
    			cv::remap(in, out, _mapX, _mapY, interp);
					cv::rotate(out, out, cv::ROTATE_90_COUNTERCLOCKWISE);
					return !out.empty();
			}
	
			double resolution, near_m, far_m, width_m;
	private:
			cv::Mat _ipmTransform;
			cv::Size _outputSize;
			bool     _useNearest;
			cv::Mat _mapX, _mapY;
};

class LaneDetector {
  public:
	LaneDetector(ros::NodeHandle &nh) : nh(nh), showflag(false), printflag(false),
		ipm_camera(nh, true)
	{
		IPM_WIDTH = ipm_camera.resolution * ipm_camera.width_m;
		IPM_HEIGHT = ipm_camera.resolution * (ipm_camera.far_m - ipm_camera.near_m);
		METER_PER_PIXEL_X = 1 / ipm_camera.resolution;
		METER_PER_PIXEL_Y = 1 / ipm_camera.resolution;
		lane_pub = nh.advertise<utils::Lane2>("/lane", 1);
		center_offset_pub = nh.advertise<std_msgs::Float32>("/lane_center_offset", 1);
		waypoints_pub = nh.advertise<std_msgs::Float32MultiArray>("/lane_waypoints", 1);
		std::string nodeName = ros::this_node::getName();
		nh.getParam(nodeName + "/showFlag", showflag);
		nh.getParam(nodeName + "/printFlag", printflag);
		nh.getParam(nodeName + "/pub", publish);
		nh.param(nodeName + "/printDuration", printDuration, false);
		nh.param(nodeName + "/newlane", newlane, false);
		nh.getParam(nodeName + "/real", real);
		if (!newlane) {
			old_lane_detector = std::make_unique<OldLaneDetector>(showflag, printflag);
		}

		double window_margin_multiplier;
		nh.getParam("window_margin_multiplier", window_margin_multiplier);
		WINDOW_MARGIN = int(window_margin_multiplier * VehicleConstants::LANE_WHITE * ipm_camera.resolution);
		nh.getParam("window_min_pixels", WINDOW_MIN_PIXELS);
		nh.getParam("window_height", window_height);
		n_windows = int((ipm_camera.far_m - ipm_camera.near_m) / window_height);
		nh.getParam("stopline_distance_threshold", stopline_distance_threshold);
		nh.getParam("pixel_to_meter_y_slope", pixel_to_meter_y_slope);
		nh.getParam("pixel_to_meter_y_intercept", pixel_to_meter_y_intercept);
		meter_to_pixel_y_slope = 1 / pixel_to_meter_y_slope;
		meter_to_pixel_y_intercept = -pixel_to_meter_y_intercept / pixel_to_meter_y_slope;
		LANE_WIDTH_PIXEL = 0.37 / METER_PER_PIXEL_X;
		LANE_WIDTH_PIXEL = LANE_WIDTH_PIXEL;

		double fx = VehicleConstants::CAMERA_PARAMS[0];
		double fy = VehicleConstants::CAMERA_PARAMS[1];
		double cx = VehicleConstants::CAMERA_PARAMS[2];
		double cy = VehicleConstants::CAMERA_PARAMS[3];
		if (real) {
			fx = VehicleConstants::CAMERA_PARAMS_REAL[0];
			fy = VehicleConstants::CAMERA_PARAMS_REAL[1];
			cx = VehicleConstants::CAMERA_PARAMS_REAL[2];
			cy = VehicleConstants::CAMERA_PARAMS_REAL[3];
		}
	}

	enum LANES { NONE = 0, LEFT = 1, BOTH = 2, RIGHT = 3};
	const double IMG_WIDTH = 640;
	const double IMG_HEIGHT = 480;
	double IPM_WIDTH = 640;
	double IPM_HEIGHT = 480;
	double METER_PER_PIXEL_X;
	double METER_PER_PIXEL_Y;
	double LANE_WIDTH_PIXEL = 0.37 / METER_PER_PIXEL_X;
	int n_windows = 9;
	double window_height = 0.2;
	int WINDOW_MARGIN = 50;
	int WINDOW_MIN_PIXELS = 50;
	double pixel_to_meter_y_slope = 0.003138337;
	double pixel_to_meter_y_intercept = 0.3778245;
	double meter_to_pixel_y_slope = 1 / pixel_to_meter_y_slope;
	double meter_to_pixel_y_intercept = -pixel_to_meter_y_intercept / pixel_to_meter_y_slope;

	IPMCamera ipm_camera;

	ros::NodeHandle nh;
	ros::Publisher lane_pub;
	ros::Publisher center_offset_pub;
	ros::Publisher waypoints_pub;
	utils::Lane2 lane_msg;

	bool showflag, printflag, newlane, printDuration, publish, real;

	std::unique_ptr<OldLaneDetector> old_lane_detector;

	// NEW LANE
	cv::Mat processed_image = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	cv::Mat ipm_color = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	cv::Mat ipm_processed = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	cv::Mat binary_image = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);

	std_msgs::Float32MultiArray waypoints_msg;
	std_msgs::MultiArrayDimension dimension;

	// LINE FIT
	double stopline_dist = -1;
	double stopline_dist_to_front = -1;
	double stopline_distance_threshold;
	bool stopline = false;
	bool cross_walk = false;
	int lane_to_fit = NONE;
	VectorXd left_fit = VectorXd(4);
	VectorXd right_fit = VectorXd(4);
	cv::Mat histogram;

	void publish_lane(const cv::Mat &image) {
		if (image.empty()) {
			ROS_WARN("empty image received in lane detector");
			return;
		}
		auto start = high_resolution_clock::now();
		if (newlane) {
			preprocess(image, processed_image);
			if (!ipm_camera.getIPM(processed_image, ipm_processed)) return;
			stopline_dist = find_stopline(ipm_processed);

			// cv::imshow("processed image", processed_image);
			// cv::imshow("processed ipm", ipm_processed);
			// cv::waitKey(1);
			// return;

			std_msgs::Float32 center_offset_msg;
			center_offset_msg.data = find_lanes(ipm_processed);
			center_offset_pub.publish(center_offset_msg);
			return;
			if(!line_fit(ipm_processed)) return;

			auto wpts = get_waypoints(left_fit, right_fit, 40, 0.032);
			// for (size_t i = 0; i + 6 <= wpts.size(); i += 3) {
			// 		float x1 = wpts[i];
			// 		float y1 = wpts[i + 1];
			// 		float x2 = wpts[i + 3];
			// 		float y2 = wpts[i + 4];
			
			// 		float dx = x2 - x1;
			// 		float dy = y2 - y1;
			// 		float distance = std::sqrt(dx * dx + dy * dy);
			
			// 		std::cout << "Distance between waypoint " << i / 3 << " and " << (i / 3 + 1)
			// 							<< " = " << distance << " meters" << std::endl;
			// }

			// for (int i = 0; i < wpts.size(); i += 3) {
			// 	std::cout << i/3 << ") x: " << wpts[i] << ", y: " << wpts[i + 1] << ", yaw: " << wpts[i + 2] << std::endl;
			// }

			waypoints_msg.data.clear();
			waypoints_msg.layout.dim.clear();
			waypoints_msg.layout.data_offset = 0;

			// Populate data array with waypoints
			for (int i = 0; i < wpts.size(); i++) {
				waypoints_msg.data.push_back(wpts[i]);
			}
			waypoints_pub.publish(waypoints_msg);

			lane_msg.center = 320.0;
			lane_msg.stopline_dist = stopline_dist_to_front;
			lane_msg.stopline = stopline;
			lane_msg.header.stamp = ros::Time::now();
			lane_pub.publish(lane_msg);

			if (showflag) {
				if (!ipm_camera.getIPM(image, ipm_color))
					return;
				cv::Mat gyu_img = viz3(ipm_color, image, wpts, true);
				cv::imshow("Binary Image", gyu_img);
				cv::waitKey(1);
			}
		} else {
			preprocess(image, processed_image);
			double center = old_lane_detector->optimized_histogram(processed_image, showflag, printflag);
			lane_msg.center = center;
			lane_msg.stopline = old_lane_detector->stopline;
			lane_msg.stopline_dist = old_lane_detector->stopline_dist;
			lane_msg.header.stamp = ros::Time::now();
			lane_pub.publish(lane_msg);
		}
		if (printDuration) {
			auto stop = high_resolution_clock::now();
			auto duration = duration_cast<microseconds>(stop - start);
			ROS_INFO("lane duration: %ld", duration.count());
		}
	}

	double pixel_to_meter_y(double pixel) { 
		return pixel * METER_PER_PIXEL_X;
	}
	double meter_to_pixel_y(double meter) { 
		return meter / METER_PER_PIXEL_X;
	}
	
	double get_derivative(double y, const VectorXd &coeffs) {
			// Compute derivative of the polynomial: f(y) = c0 + c1*y + c2*y^2 + ...
			double derivative = 0.0;
			for (int i = 1; i < coeffs.size(); ++i) {
					derivative += i * coeffs[i] * pow(y, i - 1);
			}
			return derivative;
	}

	std::vector<float> get_waypoints(const VectorXd &left_fit, const VectorXd &right_fit, int num_waypoints, double density) {
		std::vector<cv::Point2f> world_waypoints;
		world_waypoints.reserve(num_waypoints);
		std::cout << "get_waypoints" << std::endl;

		// add points from car center to first point seen in the image
		double current_y_meter = pixel_to_meter_y_intercept; // where the image starts
		double y_pixel = IMG_HEIGHT - meter_to_pixel_y(current_y_meter + 0.1);
		double left_x = evaluate_poly(y_pixel, left_fit);
		double right_x = evaluate_poly(y_pixel, right_fit);
		double center_x = 0.5 * (left_x + right_x);
		double world_x = (320 - center_x) * METER_PER_PIXEL_X;
		double car_center_to_first_waypoint = current_y_meter + VehicleConstants::REALSENSE_TF[0];
		int num_unseen_waypoints = car_center_to_first_waypoint / density;
		double slope = world_x / (pixel_to_meter_y_intercept + VehicleConstants::REALSENSE_TF[0]);
		for (int i = 0; i < num_unseen_waypoints; ++i) {
    	float y = -VehicleConstants::REALSENSE_TF[0] + i * density;
			float x = slope * i * density;
			world_waypoints.push_back(cv::Point2f(y, x));
		}

		// add points from first point seen in the image to the last point seen in the image
		current_y_meter = pixel_to_meter_y_intercept;
		std::cout << "done adding unseen waypoints, num_unseen_waypoints: " << num_unseen_waypoints << ", current_y_meter: " << current_y_meter << std::endl;
		for (int i = 0; i < num_waypoints - num_unseen_waypoints; ++i) {
			y_pixel = IMG_HEIGHT - meter_to_pixel_y(current_y_meter + 0.1);

			left_x = evaluate_poly(y_pixel, left_fit);
			right_x = evaluate_poly(y_pixel, right_fit);
			center_x = 0.5 * (left_x + right_x);
			world_x = (320 - center_x) * METER_PER_PIXEL_X;

			world_waypoints.push_back(cv::Point2f(static_cast<float>(current_y_meter), static_cast<float>(world_x))); // body-fixed frame

			// Compute derivatives dx/dy for both lanes at this y
			double left_dxdy = get_derivative(y_pixel, left_fit);
			double right_dxdy = get_derivative(y_pixel, right_fit);
			double center_dxdy = 0.5 * (left_dxdy + right_dxdy);
			// Estimate local pixel-to-meter ratio in Y direction at this y_pixel
			double meter_per_pixel_y = pixel_to_meter_y_slope;
			// Scale dx/dy from pixel space to world space: adjust slope accordingly
			double center_dxdy_world = center_dxdy * (METER_PER_PIXEL_X / meter_per_pixel_y);
			// Compute dy using arc length formula
			double dy = density / std::sqrt(1.0 + std::pow(center_dxdy_world, 2.0));
			// Advance current y (in meters) by this amount
			current_y_meter += dy;
		}

		std::cout << "done adding seen waypoints" << std::endl;

		// add VehicleConstants::REALSENSE_TF[0] to x values so that origin is center of car instead of camera
		for (auto &waypoint : world_waypoints) {
			waypoint.x += VehicleConstants::REALSENSE_TF[0];
		}

		// 2. Compute yaw angles between successive waypoints.
		std::vector<float> yaw_values;
		for (size_t i = 0; i < world_waypoints.size() - 1; ++i) {
			float delta_x = world_waypoints[i + 1].x - world_waypoints[i].x;
			float delta_y = world_waypoints[i + 1].y - world_waypoints[i].y;
			// Use atan2 to compute the yaw angle (in radians).
			float yaw = std::atan2(delta_y, delta_x);
			yaw_values.push_back(yaw);
		}
		// For the last waypoint, replicate the previous yaw value (or set to 0 if none exist).
		if (!yaw_values.empty()) {
			yaw_values.push_back(yaw_values.back());
		} else {
			yaw_values.push_back(0.0f);
		}

		// 3. Combine the x, y, and yaw values into a single flat vector.
		std::vector<float> waypoints_with_yaw;
		for (size_t i = 0; i < world_waypoints.size(); ++i) {
			waypoints_with_yaw.push_back(world_waypoints[i].x);
			waypoints_with_yaw.push_back(world_waypoints[i].y);
			waypoints_with_yaw.push_back(yaw_values[i]);
		}

		return waypoints_with_yaw;
	}

	double find_stopline(const cv::Mat& image) {
			const int img_height = image.rows;
			const int img_width = image.cols;
			stopline_dist = -1;
			double stop_loc = -1.0;
			bool found = false;
			cv::Mat horistogram;
			cv::reduce(image, horistogram, 1, cv::REDUCE_SUM, CV_32S);

			std::vector<int> hist;
			for (int i = 0; i < horistogram.rows; ++i) {
			// for (int i = horistogram.rows-1; i > 0; --i) {
					hist.push_back(static_cast<int>(horistogram.at<int>(0,i)/255));
					if (hist[i] >= LANE_WIDTH_PIXEL * 0.753) {
							stop_loc = i;
							found = true;
							break;
					}
			}
			if (!found) {
					stopline_dist = -1;
					return stopline_dist;
			}
			// stopline_dist = (img_height - stop_loc) * METER_PER_PIXEL_Y;
			double pixel_value = (img_height - stop_loc);
			stopline_dist = pixel_to_meter_y(pixel_value);
			stopline_dist_to_front = stopline_dist;
			if (real) {
				stopline_dist_to_front += VehicleConstants::REALSENSE_TF_REAL[0];
			} else {
				stopline_dist_to_front += VehicleConstants::REALSENSE_TF[0];
			}
			stopline_dist_to_front -= VehicleConstants::CAR_LENGTH / 2;
			if (stopline_dist_to_front > 0.5) {
				stopline = true;
			} else {
				stopline = false;
				stopline_dist = -1;
				stopline_dist_to_front = -1;
			}
			return stopline_dist;
	}

	bool preprocess(const cv::Mat &inputImage, cv::Mat &outputImage) {
		if (inputImage.empty()) {
				return false;
		}

		cv::cvtColor(inputImage, outputImage, cv::COLOR_BGR2GRAY); // Convert to grayscale
		cv::GaussianBlur(outputImage, outputImage, cv::Size(5, 5), 0); // Apply Gaussian blur

		static bool first;
		static int adaptive_threshold_block_size = 199;
		static int adaptive_threshold_c = -20;
		if (first) {
			nh.getParam("adaptive_threshold_block_size", adaptive_threshold_block_size);
			nh.getParam("adaptive_threshold_c", adaptive_threshold_c);
			first = false;
		}
		cv::adaptiveThreshold(
				outputImage,
				outputImage,
				255,
				// cv::ADAPTIVE_THRESH_GAUSSIAN_C,
				cv::ADAPTIVE_THRESH_MEAN_C,
				cv::THRESH_BINARY,
				adaptive_threshold_block_size,
				adaptive_threshold_c
		);

		// Apply morphological operations to remove noise
		// static cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(3, 3));
		// cv::morphologyEx(outputImage, outputImage, cv::MORPH_OPEN, kernel);
		// cv::morphologyEx(outputImage, outputImage, cv::MORPH_CLOSE, kernel);

		// Create a mask to remove the top part of the image
		// static cv::Mat mask;
		// mask = cv::Mat::zeros(outputImage.size(), outputImage.type());
		// int roiStartRow = static_cast<int>(outputImage.rows / 2.5); // Start ROI at 40% of the image height
		// mask(cv::Range(roiStartRow, outputImage.rows), cv::Range::all()) = 255;
		// cv::bitwise_and(outputImage, mask, outputImage);

		// int h = outputImage.rows;
		// int w = outputImage.cols;

		// // Create a mask to remove the car hood
		// int x_start = static_cast<int>(w * 0.27);
		// int x_end   = static_cast<int>(w * 0.73);
		// int y_start = static_cast<int>(h * 0.92);
		// int y_end   = static_cast<int>(h * 1);
		// cv::Rect carMaskROI(x_start, y_start, x_end - x_start, y_end - y_start);
		// outputImage(carMaskROI) = 0;

		// cv::imshow("outputImage", outputImage);
		// cv::waitKey(1);
		return !outputImage.empty();
	}

	std::vector<int> lane_indices;
	void extract_lanes(const cv::Mat& hist_data) {
			lane_indices.clear();
			if (hist_data.empty()) return;
			// ─── tunable constants ─────────────────────────────────────────
			constexpr int THRESH           = 1500;  // histogram‐sum threshold
			constexpr int MIN_LANE_WIDTH   = 5;     // min run width in px
			constexpr int CLUSTER_GAP      = 20;    // merge midpoints closer than this
			// ────────────────────────────────────────────────────────────────
			const int W = hist_data.cols;
			bool inLane = false;
			int start = 0;
			// 1) find rising/falling edges, record midpoints
			for (int x = 0; x < W; ++x) {
					int v = hist_data.at<int>(0, x);
					if (!inLane && v >= THRESH) {
							inLane = true;
							start  = x;
					}
					else if (inLane && v < THRESH) {
							int end = x - 1;
							inLane = false;
							if (end - start + 1 >= MIN_LANE_WIDTH) {
									lane_indices.push_back((start + end) / 2);
							}
					}
			}
			// if a lane runs right to the edge:
			if (inLane) {
					int end = W - 1;
					if (end - start + 1 >= MIN_LANE_WIDTH) {
							lane_indices.push_back((start + end) / 2);
					}
			}
			// 2) merge any midpoints that are too close together
			if (lane_indices.size() > 1) {
					std::sort(lane_indices.begin(), lane_indices.end());
					std::vector<int> merged;
					int sum   = lane_indices[0];
					int count = 1;
					for (size_t i = 1; i < lane_indices.size(); ++i) {
							if (lane_indices[i] - lane_indices[i-1] <= CLUSTER_GAP) {
									sum   += lane_indices[i];
									count += 1;
							} else {
									merged.push_back(sum / count);
									sum   = lane_indices[i];
									count = 1;
							}
					}
					merged.push_back(sum / count);
					lane_indices.swap(merged);
			}
	}
	double find_lanes(const cv::Mat &binaryIPM)
	{
			lane_indices.clear();
			if (binaryIPM.empty()) return -1.0;

			constexpr float ROI_FRACTION   = 0.10f; // analyse only bottom 10 % of image
			constexpr float FRACTION_FROM_BOT = 0.06f; // start 6 % above bottom edge

			const int H = binaryIPM.rows, W = binaryIPM.cols;
			const int y0     = static_cast<int>((1.0f - ROI_FRACTION - FRACTION_FROM_BOT) * H);
			const int height = static_cast<int>(ROI_FRACTION * H);
			const int width  = static_cast<int>(0.9f * W);               // crop 5 % per side

			const cv::Rect roiRect(static_cast<int>((W - width) / 2), y0, width, height);
			const cv::Mat  roi = binaryIPM(roiRect);

			cv::Mat hist;
			cv::reduce(roi, hist, 0, cv::REDUCE_SUM, CV_32S);
			extract_lanes(hist);                                    // fills lane_indices

			cv::Mat vis;                                           // colour copy for drawing
			if (true)
			{
					cv::cvtColor(binaryIPM, vis, cv::COLOR_GRAY2BGR);   // → BGR so we can colour
					cv::rectangle(vis, roiRect, cv::Scalar(255, 0, 0), 2);
					for (int x : lane_indices)
					{
							cv::line(vis,
											cv::Point(x, roiRect.y),
											cv::Point(x, roiRect.y + roiRect.height),
											cv::Scalar(0, 255, 255), 2);
					}
			}

			double offset_from_center = -1.0;                       // default: fail

			if (lane_indices.size() == 2 && lane_indices[0] > 0 && lane_indices[1] > 0)
			{
					const int diff = lane_indices[1] - lane_indices[0];
					if (diff > 0 && std::abs(diff - LANE_WIDTH_PIXEL) < 0.15 * LANE_WIDTH_PIXEL)
					{
							offset_from_center = -(lane_indices[0] + lane_indices[1] - W) / 2.0 * METER_PER_PIXEL_X;

							if (showflag)
							{
									const int midX = (lane_indices[0] + lane_indices[1]) / 2;
									const int midY = roiRect.y + roiRect.height / 2;
									cv::circle(vis, cv::Point(midX, midY), 8, cv::Scalar(0, 255, 0), -1);
									char buf[64];
									std::snprintf(buf, sizeof(buf), "offset: %.2fm", offset_from_center);
									cv::putText(vis, buf, cv::Point(midX + 10, midY - 10),
															cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
							}
					}
			}

			if (true)
			{
					cv::imshow("Lane Visualisation", vis);
					cv::waitKey(1);
			}

			return offset_from_center;
	}

	std::vector<int> find_closest_pair(const std::vector<int> &indices, int LANE_WIDTH_PIXEL) {
		int n = indices.size(); // size of input array

		if (n < 2) { // check to see if at least two lane lines
			throw std::invalid_argument("Array must have at least two elements");
		}

		int min_diff = std::numeric_limits<int>::max();
		std::vector<int> result_pair(2);
		result_pair[0] = indices[0];
		result_pair[1] = indices[1];

		for (int i = 0; i < n - 1; ++i) { // iterate over different pairs
			for (int j = i + 1; j < n; ++j) {
				int current_diff = std::abs(std::abs(indices[i] - indices[j]) - LANE_WIDTH_PIXEL); // check for how close to optimal distance the current distance is
				if (current_diff < min_diff) {
					min_diff = current_diff; // compare current pair difference with optimal difference
					result_pair[0] = indices[i];
					result_pair[1] = indices[j];
				}
			}
		}
		return result_pair;
	}

	double binomial(int n, int k) { return std::tgamma(n + 1) / (std::tgamma(k + 1) * std::tgamma(n - k + 1)); }

	void polyfit(VectorXd &x, VectorXd &y_raw, VectorXd &coeffs, int order = 3) {
		const int n = y_raw.size();
		const int k = order + 1;

		// 1. Normalize y-values to [-1, 1] range
		const double y_mean = y_raw.mean();
		const double y_std = std::sqrt((y_raw.array() - y_mean).square().mean());
		const VectorXd y = (y_raw.array() - y_mean) / y_std;

		// 2. Initialize weights and design matrix
		VectorXd weights = VectorXd::Ones(n);
		MatrixXd X(n, k);
		X.col(0) = VectorXd::Ones(n);

		// 3. Build Vandermonde matrix with normalized y
		for (int j = 1; j < k; ++j) {
			X.col(j) = X.col(j - 1).cwiseProduct(y);
		}

		// 4. Least squares
		MatrixXd XW = X.array().colwise() * weights.array();
		coeffs = (XW.transpose() * X).ldlt().solve(XW.transpose() * x);

		// 5. Denormalize coefficients to original y scale
		VectorXd normalized_coeffs = coeffs;
		coeffs = VectorXd::Zero(k);

		for (int j = 0; j < k; ++j) {
			const double scale = std::pow(y_std, -j);
			for (int i = 0; i <= j; ++i) {
				const double binom = binomial(j, i);
				coeffs[i] += normalized_coeffs[j] * std::pow(-y_mean, j - i) * scale * binom;
			}
		}
	}

	double evaluate_poly(double y, const VectorXd &coeffs) {
		// Horner's method: x = c0 + y*(c1 + y*(c2 + y*(c3 + ...)))
		double x = 0.0;
		for (int i = coeffs.size() - 1; i >= 0; --i) {
			x = x * y + coeffs[i];
		}
		return x;
	}

	void scale_poly(VectorXd &coeffs, double scale) {
		for (int i = 0; i < coeffs.size(); ++i) {
			coeffs[i] *= std::pow(scale, i - 1);
		}
	}

	bool line_fit(const cv::Mat &binary_warped) {
		const int img_height = binary_warped.rows;
		const int img_width = binary_warped.cols;
		static int leftx_base = 0;
		static int rightx_base = img_width;
		lane_to_fit = NONE;

		int size_indices = lane_indices.size(); // Number of lanes detected

		if (size_indices == 0) { // Check to see if lanes detected, if not return
			lane_to_fit = NONE;
			return false;
		}

		if (size_indices == 1) {						  // If only one lane line is detected
			if (lane_indices[0] < img_width/2) { // Check on which side of the car it is
				lane_to_fit = LEFT;						  // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
				leftx_base = lane_indices[0];
				rightx_base = 0;
			} else {
				lane_to_fit = RIGHT; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
				leftx_base = 0;
				rightx_base = lane_indices[0];
			}
		} else {
			if (size_indices > 2) { // If more than two lane lines are detected
				std::vector<int> closest_pair = find_closest_pair(lane_indices, LANE_WIDTH_PIXEL);
				lane_indices[0] = closest_pair[0]; // Initialize the start of the lane line at bottom of the screen
				lane_indices[1] = closest_pair[1];
			}
			int delta = std::abs(lane_indices[0] - lane_indices[1]); // Check to see if the two lane lines are close enough to be the same
			if (delta < img_width/4) {
				lane_indices[0] = 0.5 * (lane_indices[0] + lane_indices[1]);
				if (lane_indices[0] < img_width/2) {
					lane_to_fit = LEFT; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
					leftx_base = lane_indices[0];
					rightx_base = 0;
				} else {
					lane_to_fit = RIGHT; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
					leftx_base = 0;
					rightx_base = lane_indices[0];
				}
			} else {
				leftx_base = lane_indices[0]; // Initialize the start of the lane line at bottom of the screen
				rightx_base = lane_indices[1];
				lane_to_fit = BOTH; // Set number of fits as a reference
			}
		}

		cv::Mat out_img = cv::Mat::zeros(binary_warped.size(), CV_8UC3);
		int window_height = static_cast<int>(binary_warped.rows / n_windows); // Caclulate height of parsing windows
		static std::vector<cv::Point> nonzero;

		cv::findNonZero(binary_warped, nonzero);

		std::vector<int> nonzeroy, nonzerox;
		for (size_t i = 0; i < nonzero.size(); i += 2) { // Increment index by 2
			nonzeroy.push_back(nonzero[i].y);
			nonzerox.push_back(nonzero[i].x);
		}

		int leftx_current = leftx_base;
		int rightx_current = rightx_base;

		std::vector<int> left_lane_inds;
		std::vector<int> right_lane_inds;

		int mean_right_prev = 0;
		int mean_left_prev = 0;
		bool right_done = false;
		bool left_done = false;
		int num_left_windows = 0;
		int num_right_windows = 0;
		for (int window = 0; window < n_windows; ++window) {
			if (left_done && right_done) {
				break;
			}
			int win_y_low = binary_warped.rows - (window + 1) * window_height;
			int win_y_high = binary_warped.rows - window * window_height;

			// LEFT LANE
			if (!left_done && (lane_to_fit == LEFT || lane_to_fit == BOTH)) {
				int win_xleft_low = leftx_current - WINDOW_MARGIN; // Bounding boxes around the lane lines
				int win_xleft_high = leftx_current + WINDOW_MARGIN;
				
				// DISPLAY
				cv::rectangle(out_img, cv::Point(win_xleft_low, win_y_low),
                  cv::Point(win_xleft_high, win_y_high),
                  cv::Scalar(0, 255, 0), 2);

				int sum_left = 0;
				std::vector<int> good_left_inds;
				for (size_t i = 0; i < nonzerox.size(); ++i) { // Parse through and only select pixels within the bounding boxes
					if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high && nonzerox[i] >= win_xleft_low && nonzerox[i] < win_xleft_high) {
						good_left_inds.push_back(i); // Keep pixels within the boxes
						sum_left += nonzerox[i];
					}
				}

				if (good_left_inds.size() > WINDOW_MIN_PIXELS) { // Recenter mean for the next bounding box
					int mean_left = sum_left / good_left_inds.size();
					if (mean_left - WINDOW_MARGIN/2 > img_width || mean_left + WINDOW_MARGIN/2 < 0) {
						// std::cout << "left lane done at window: " << window << ", mean_left: " << mean_left << ", leftx_current: " << leftx_current << std::endl;
						left_done = true;
					} else {
						num_left_windows++;
						left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end()); // Append all good indices together
						// DISPLAY
						cv::circle(out_img, cv::Point(mean_left, (win_y_low + win_y_high) / 2), 10, cv::Scalar(0, 255, 255), -1);
						int delta_left = mean_left- mean_left_prev;
						mean_left_prev = mean_left;
						// std::cout << window << ") mean_left: " << mean_left << ", leftx_current: " << leftx_current << ", delta_left: " << delta_left << ", size: " << good_left_inds.size() << std::endl;
						if (window == 0) {
							leftx_current = mean_left;
						} else {
							leftx_current = leftx_current + (delta_left * 1.5);
						}
					}
				} else {
					// std::cout << "left lane done at window: " << window << ", good_left_inds.size(): " << good_left_inds.size() << ", min pixels: " << WINDOW_MIN_PIXELS << std::endl;
					left_done = true;
				}
			}

			// RIGHT LANE
			if (!right_done && (lane_to_fit == RIGHT || lane_to_fit == BOTH)) {
				int win_xright_low = rightx_current - WINDOW_MARGIN; // Bounding boxes around the lane lines
				int win_xright_high = rightx_current + WINDOW_MARGIN;

				// DISPLAY
				cv::rectangle(out_img, cv::Point(win_xright_low, win_y_low),
									cv::Point(win_xright_high, win_y_high),
									cv::Scalar(0, 255, 0), 2);  // Green for right lane window

				int sum_right = 0;
				std::vector<int> good_right_inds; // x values of pixels within the bounding boxes
				for (size_t i = 0; i < nonzerox.size(); ++i) { // Parse through and only select pixels within the bounding boxes
					if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high && nonzerox[i] >= win_xright_low && nonzerox[i] < win_xright_high) {
						good_right_inds.push_back(i); // Keep pixels within the boxes
						sum_right += nonzerox[i]; // Sum of all x values within the boxes that are nonzero
					}
				}

				if (good_right_inds.size() > WINDOW_MIN_PIXELS) { // Keep pixels within the boxes
					int mean_right = sum_right / good_right_inds.size();
					if (mean_right - WINDOW_MARGIN/2 > img_width || mean_right + WINDOW_MARGIN/2 < 0) {
						right_done = true;
					} else {
						num_right_windows++;
						right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end()); // Append all good indices together
						// DISPLAY
						cv::circle(out_img, cv::Point(mean_right, (win_y_low + win_y_high) / 2), 10, cv::Scalar(0, 255, 255), -1);
						int delta_right = mean_right - mean_right_prev;
						mean_right_prev = mean_right;
						// std::cout << window << ") mean_right: " << mean_right << ", rightx_current: " << rightx_current << ", delta_right: " << delta_right << ", size: " << good_right_inds.size() << std::endl;
						if (window == 0) {
							rightx_current = mean_right;
						} else {
							rightx_current = rightx_current + (delta_right * 1.5);
						}
					}
				} else {
					right_done = true;
				}
			}
		}
		// DISPLAY
		for (int i = 0; i < left_lane_inds.size(); ++i) {
			cv::circle(out_img, cv::Point(nonzerox[left_lane_inds[i]], nonzeroy[left_lane_inds[i]]), 2, cv::Scalar(0, 0, 255), -1);
		}
		for (int i = 0; i < right_lane_inds.size(); ++i) {
			cv::circle(out_img, cv::Point(nonzerox[right_lane_inds[i]], nonzeroy[right_lane_inds[i]]), 2, cv::Scalar(255, 0, 0), -1);
		}
		cv::imshow("Detected Pixels", out_img);
		cv::waitKey(1);

		// std::cout << "num_left_windows: " << num_left_windows << ", num_right_windows: " << num_right_windows << std::endl;
		// Declare vectors to contain the pixel coordinates to fit
		VectorXd leftx;
		VectorXd lefty;
		VectorXd rightx;
		VectorXd righty;

		if (lane_to_fit == LEFT && num_left_windows > 2) { // left good
			fit_points(leftx, lefty, left_lane_inds, nonzeroy, nonzerox, left_fit);
			right_fit = left_fit;
			right_fit(0) += LANE_WIDTH_PIXEL;
		} else if (lane_to_fit == RIGHT && num_right_windows > 2) { // right good
			fit_points(rightx, righty, right_lane_inds, nonzeroy, nonzerox, right_fit);
			left_fit = right_fit;
			left_fit(0) -= LANE_WIDTH_PIXEL;
		} else if (lane_to_fit == BOTH) { // both good
			if (num_left_windows > 2 && num_right_windows > 2) {
				fit_points(leftx, lefty, left_lane_inds, nonzeroy, nonzerox, left_fit);
				fit_points(rightx, righty, right_lane_inds, nonzeroy, nonzerox, right_fit);
			} else if (num_left_windows > 2) { // left good
				fit_points(leftx, lefty, left_lane_inds, nonzeroy, nonzerox, left_fit);
				right_fit = left_fit;
				right_fit(0) += LANE_WIDTH_PIXEL;
			} else if (num_right_windows > 2) { // right good
				fit_points(rightx, righty, right_lane_inds, nonzeroy, nonzerox, right_fit);
				left_fit = right_fit;
				left_fit(0) -= LANE_WIDTH_PIXEL;
			} else { // both bad
				return false;
			}
		} 
		return true;
	}

	void fit_points(VectorXd& x, VectorXd& y, const std::vector<int>& indices, 
		const std::vector<int>& nonzeroy, const std::vector<int>& nonzerox, VectorXd& fit) 
	{
		const int num_points = indices.size();
		x.resize(num_points);
		y.resize(num_points);

		// Fill using Eigen vector indexing
		for (int i = 0; i < num_points; ++i) {
			const int idx = indices[i];
			x(i) = nonzerox[idx];
			y(i) = nonzeroy[idx];
		}
		polyfit(x, y, fit);
	}

	cv::Mat viz3(const cv::Mat &binary_warped, const cv::Mat &non_warped, const std::vector<float> waypoints, bool IPM = true) {
		const int img_height = binary_warped.rows;
		const int img_width = binary_warped.cols;
		// Generate y values for plotting
		std::vector<double> ploty;
		for (int i = 0; i < binary_warped.rows; ++i) {
			ploty.push_back(i);
		}

		// Create an empty image
		cv::Mat result(binary_warped.size(), CV_8UC3, cv::Scalar(0, 0, 0));

		// Update values only if they are not None
		std::vector<double> left_fitx, right_fitx;
		if (lane_to_fit == LEFT || lane_to_fit == BOTH) {
			for (double y : ploty) {
				left_fitx.push_back(evaluate_poly(y, left_fit));
			}
			std::vector<cv::Point> left_points;
			for (size_t i = 0; i < left_fitx.size(); ++i) {
				left_points.push_back(cv::Point(left_fitx[i], ploty[i]));
			}
			cv::polylines(result, left_points, false, cv::Scalar(255, 255, 0), 15);
		}
		if (lane_to_fit == RIGHT || lane_to_fit == BOTH) {
			for (double y : ploty) {
				right_fitx.push_back(evaluate_poly(y, right_fit));
			}
			std::vector<cv::Point> right_points;
			for (size_t i = 0; i < right_fitx.size(); ++i) {
				right_points.push_back(cv::Point(right_fitx[i], ploty[i]));
			}
			cv::polylines(result, right_points, false, cv::Scalar(255, 255, 0), 15);
		}

		for (int i = 0; i < waypoints.size(); i += 3) {
			int x = 320 - static_cast<int>(waypoints[i + 1] / METER_PER_PIXEL_X);
			// int y = img_height - static_cast<int>(waypoints[i] / METER_PER_PIXEL_Y);
			int y = img_height - static_cast<int>(meter_to_pixel_y(waypoints[i] - VehicleConstants::REALSENSE_TF[0]));
			cv::circle(result, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
		}

		// // Draw stop line
		if (stopline_dist > 0) {
			double stopline_y = img_height - meter_to_pixel_y(stopline_dist);
			cv::line(result, cv::Point(0, stopline_y), cv::Point(img_width, stopline_y), cv::Scalar(0, 255, 0), 5);
		}
		if (IPM) {
			cv::addWeighted(result, 0.95, binary_warped, 0.3, 0, result);
		}

		cv::resize(result, result, cv::Size(img_width, img_height), 0, 0, cv::INTER_CUBIC);

		if (stopline_dist > 0) {
			cv::putText(result, "stop:" + std::to_string(stopline_dist) + " m", cv::Point(0, 48), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
			cv::putText(result, "2front:" + std::to_string(stopline_dist_to_front) + " m", cv::Point(0, 48*4), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
		}

		return result;
	}
};
