#pragma once

#include "OldLaneDetector.hpp"
#include "stdafx.h"
#include "utils/Lane2.h"
#include "utils/constants.h"
#include <Eigen/Dense>
#include <algorithm>
#include <chrono>
#include <cmath>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>
#include <iostream>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>
#include <vector>

using namespace std::chrono;

class LaneDetector {
  public:
	LaneDetector(ros::NodeHandle &nh) : nh(nh), showflag(false), printflag(false) {
		lane_pub = nh.advertise<utils::Lane2>("/lane", 1);
		waypoints_pub = nh.advertise<std_msgs::Float32MultiArray>("/lane_waypoints", 1);
		std::string nodeName = ros::this_node::getName();
		nh.getParam(nodeName + "/showFlag", showflag);
		nh.getParam(nodeName + "/printFlag", printflag);
		nh.getParam(nodeName + "/pub", publish);
		nh.param(nodeName + "/printDuration", printDuration, false);
		nh.param(nodeName + "/newlane", newlane, false);
		if (!newlane) {
			old_lane_detector = std::make_unique<OldLaneDetector>(showflag, printflag);
		}
		y_Values.clear();
		for (int i = 0; i < 50; i++) {
			y_Values.push_back(300 + i * 25);
		};
	}

	const double PIXEL_X_TO_METERS = 0.4 / 420;
	const double PIXEL_Y_TO_METERS = 0.37 / 254;

	std::vector<float> getWorldWaypointsWithYaw(double start_y, const std::vector<double> &left_fit, const std::vector<double> &right_fit, int num_waypoints, double density) {
		std::vector<cv::Point2f> world_waypoints;
		world_waypoints.reserve(num_waypoints);

		// 1. Compute world coordinates from pixel samples.
		for (int i = 0; i < num_waypoints; ++i) {
			// Sample the y coordinate in pixel space.
			// double y_pixel = start_y + i * density / PIXEL_Y_TO_METERS;
			double y_pixel = 480 - (start_y + i * density / PIXEL_Y_TO_METERS);

			// Evaluate the left and right lane polynomials.
			double left_x = left_fit[0] + left_fit[1] * y_pixel + left_fit[2] * y_pixel * y_pixel;
			double right_x = right_fit[0] + right_fit[1] * y_pixel + right_fit[2] * y_pixel * y_pixel;

			// Use the midpoint between the lanes as the waypoint's x coordinate.
			double waypoint_x = 0.5 * (left_x + right_x);

			// Convert pixel coordinates to world coordinates.
			double world_x = (320 - waypoint_x) * PIXEL_X_TO_METERS;
			// double world_y = y_pixel * PIXEL_Y_TO_METERS;
			double world_y = density * (i + 1);

			world_waypoints.push_back(cv::Point2f(static_cast<float>(world_y), static_cast<float>(world_x))); // body-fixed frame
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

	ros::NodeHandle nh;
	ros::Publisher lane_pub;
	ros::Publisher waypoints_pub;
	utils::Lane2 lane_msg;

	bool showflag, printflag, newlane, printDuration, publish;

	std::unique_ptr<OldLaneDetector> old_lane_detector;

	void publish_lane(const cv::Mat &image) {
		if (image.empty()) {
			ROS_WARN("empty image received in lane detector");
			return;
		}
		auto start = high_resolution_clock::now();
		if (newlane) {
			// lane_detection(image);
			if (!maps_initialized) {
				initializeMaps(cameraMatrix, distCoeff, transMatrix_scaled, image.size());
			}
			cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
			if (!getIPM(grayscale_image, ipm_grayscale))
				return;
			getLanes(ipm_grayscale, binary_image);
			bool success = line_fit(binary_image);

			auto wpts = getWorldWaypointsWithYaw(0, left_fit, right_fit, 40, 0.032);
			for (int i = 0; i < wpts.size() / 3; i += 3) {
				std::cout << i << ") x: " << wpts[i] << ", y: " << wpts[i + 1] << ", yaw: " << wpts[i + 2] << std::endl;
			}

			waypoints_msg.data.clear();
			waypoints_msg.layout.dim.clear();
			waypoints_msg.layout.data_offset = 0;

			// Populate data array with waypoints
			for (int i = 0; i < wpts.size(); i++) {
				waypoints_msg.data.push_back(wpts[i]);
			}
			waypoints_pub.publish(waypoints_msg);

			static std::vector<double> waypoints;
			waypoints = getWaypoints(y_Values);

			lane_msg.center = waypoints[5];
			lane_msg.stopline = stop_loc;
			lane_msg.header.stamp = ros::Time::now();
			lane_pub.publish(lane_msg);

			if (showflag) {
				if (!getIPMFull(image, ipm_color))
					return;
				cv::Mat gyu_img = viz3(ipm_color, image, wpts, true);
				cv::imshow("Binary Image", gyu_img);
				cv::waitKey(1);
			}
		} else {
			double center = old_lane_detector->optimized_histogram(image, showflag, printflag);
			lane_msg.center = center;
			lane_msg.stopline = old_lane_detector->stopline_dist;
			lane_msg.header.stamp = ros::Time::now();
			lane_pub.publish(lane_msg);
		}
		if (printDuration) {
			auto stop = high_resolution_clock::now();
			auto duration = duration_cast<microseconds>(stop - start);
			ROS_INFO("duration: %ld", duration.count());
		}
	}

	// NEW LANE
	std::vector<int> y_Values = {650, 625, 600, 575, 550, 525, 500, 475, 450, 425, 400, 375, 350, 325, 300};
	std::vector<double> waypoints;
    const double scale_factor = 0.40;
	cv::Mat grayscale_image = cv::Mat::zeros(480 * scale_factor, 640 * scale_factor, CV_8UC1);
	cv::Mat ipm_color = cv::Mat::zeros(480 * scale_factor, 640 * scale_factor, CV_8UC3);
	cv::Mat ipm_grayscale = cv::Mat::zeros(480 * scale_factor, 640 * scale_factor, CV_8UC1);
	cv::Mat binary_image = cv::Mat::zeros(480 * scale_factor, 640 * scale_factor, CV_8UC1);

	std_msgs::Float32MultiArray waypoints_msg;
	std_msgs::MultiArrayDimension dimension;

	// LINE FIT
	int stop_loc = -1;
	int number_of_fits = 0;
	std::vector<double> left_fit = {0.0};
	std::vector<double> right_fit = {0.0};
	bool stop_line = false;
	int stop_index = 0;
	bool cross_walk = false;
	cv::Mat histogram;

	cv::Mat map1, map2;
	bool maps_initialized = false;

	// Define initial coordinates of input image as a constant global variable
	const cv::Mat initial = (cv::Mat_<float>(4, 2) << 0, 300, 640, 300, 0, 480, 640, 480);
	const cv::Mat initial_scaled = (cv::Mat_<float>(4, 2) << 0, 300 * scale_factor, 640 * scale_factor, 300 * scale_factor, 0, 480 * scale_factor, 640 * scale_factor, 480 * scale_factor);

	// Define where the initial coordinates will end up on the final image as a constant global variable
	const cv::Mat final = (cv::Mat_<float>(4, 2) << 0, 0, 640, 0, 0, 480, 640, 480);
	const cv::Mat final_scaled = (cv::Mat_<float>(4, 2) << 0, 0, 640 * scale_factor, 0, 0, 480 * scale_factor, 640 * scale_factor, 480 * scale_factor);

	// Compute the transformation matrix
	cv::Mat transMatrix = cv::getPerspectiveTransform(initial, final);
	cv::Mat invMatrix = cv::getPerspectiveTransform(final, initial);

	cv::Mat transMatrix_scaled = cv::getPerspectiveTransform(initial_scaled, final_scaled);
	cv::Mat invMatrix_scaled = cv::getPerspectiveTransform(final_scaled, initial_scaled);

	double fx = VehicleConstants::CAMERA_PARAMS[0];
	double fy = VehicleConstants::CAMERA_PARAMS[1];
	double cx = VehicleConstants::CAMERA_PARAMS[2];
	double cy = VehicleConstants::CAMERA_PARAMS[3];
	const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) << fx, 0, cx, 0, fy, cy, 0, 0, 1);

	const cv::Mat distCoeff = cv::Mat();

	void initializeMaps(const cv::Mat &cameraMatrix, const cv::Mat &distCoeff, const cv::Mat &transMatrix, const cv::Size &size) {
		cv::initUndistortRectifyMap(cameraMatrix, distCoeff, cv::Mat(), cameraMatrix, size, CV_16SC2, map1, map2);
		maps_initialized = true;
	}

	// NEW LANE
	int getIPM(const cv::Mat &in, cv::Mat &output) {
		static cv::Mat remapped;
        cv::Mat input;
        cv::resize(in, input, cv::Size(), scale_factor, scale_factor, cv::INTER_NEAREST);
		cv::remap(input, remapped, map1, map2, cv::INTER_LINEAR);
		cv::warpPerspective(remapped, output, transMatrix_scaled, input.size(), cv::INTER_LINEAR);
		return !output.empty();
	}

	int getIPMFull(const cv::Mat &input, cv::Mat &output) {
		static cv::Mat remapped;
		cv::remap(input, remapped, map1, map2, cv::INTER_LINEAR);
		cv::warpPerspective(remapped, output, transMatrix, input.size(), cv::INTER_LINEAR);
		return !output.empty();
	}

	void getLanes(const cv::Mat &inputImage, cv::Mat &outputImage) {
		static cv::Mat imageHist;
		cv::calcHist(&inputImage, 1, 0, cv::Mat(), imageHist, 1, &inputImage.rows, 0);
		double minVal, maxVal;
		cv::minMaxLoc(imageHist, &minVal, &maxVal);
		int threshold_value = std::min(std::max(static_cast<int>(maxVal) - 75, 30), 200);
		static cv::Mat binary_thresholded = cv::Mat::zeros(inputImage.size(), CV_8UC1);
		cv::threshold(inputImage, binary_thresholded, threshold_value, 255, cv::THRESH_BINARY);
		outputImage = binary_thresholded;
	}

	std::vector<double> getWaypoints(std::vector<int> &y_Values) {
		int offset = 175;
		std::vector<double> wayPoint(y_Values.size()); // Resized wayPoint to match the size of y_Values
		std::vector<double> L_x(y_Values.size());	   // Resized L_x
		std::vector<double> R_x(y_Values.size());	   // Resized R_x
		auto &fit_L = left_fit;
		auto &fit_R = right_fit;
		if (number_of_fits == 2) {
			for (size_t i = 0; i < y_Values.size(); ++i) {
				L_x[i] = fit_L[0] + y_Values[i] * fit_L[1] + fit_L[2] * (y_Values[i]) * (y_Values[i]);
				R_x[i] = fit_R[0] + y_Values[i] * fit_R[1] + fit_R[2] * (y_Values[i]) * (y_Values[i]);
				wayPoint[i] = 0.5 * (L_x[i] + R_x[i]);
				wayPoint[i] = static_cast<int>(std::max(0.0, std::min(static_cast<double>(wayPoint[i]), 639.0)));
			}
		} else if (number_of_fits == 1) {
			for (size_t i = 0; i < y_Values.size(); ++i) {
				L_x[i] = (offset - (480 - y_Values[i]) * 0.05) + fit_L[0] + y_Values[i] * fit_L[1] + fit_L[2] * (y_Values[i]) * (y_Values[i]);
				wayPoint[i] = std::max(0.0, std::min(L_x[i], 639.0));
			}
		} else if (number_of_fits == 3) {
			for (size_t i = 0; i < y_Values.size(); ++i) {
				R_x[i] = -(offset - (480 - y_Values[i]) * 0.08) + fit_R[0] + y_Values[i] * fit_R[1] + fit_R[2] * (y_Values[i]) * (y_Values[i]);
				wayPoint[i] = std::max(0.0, std::min(R_x[i], 639.0));
			}
		} else {
			for (size_t i = 0; i < y_Values.size(); ++i) {
				wayPoint[i] = 320;
			}
		}
		return wayPoint;
	}

	std::vector<int> center_indices;
	void find_center_indices(const cv::Mat &histogram, int threshold) {
		static std::vector<int> hist;
		hist.clear();
		for (int i = 0; i < histogram.cols; ++i) {
			hist.push_back(histogram.at<int>(0, i));
		}

		double mean = std::accumulate(hist.begin(), hist.end(), 0.0) / hist.size();

		double variance = 0.0;
		for (int value : hist) {
			variance += std::pow(value - mean, 2);
		}
		double stddev = std::sqrt(variance / hist.size());

		int adaptive_threshold = static_cast<int>(mean + 0.5 * stddev);
		std::vector<int> above_threshold;
		for (int i = 0; i < hist.size(); ++i) {
			if (hist[i] > adaptive_threshold) {
				above_threshold.push_back(i);
			}
		}

		std::vector<std::vector<int>> consecutive_groups;
		for (int i = 0; i < above_threshold.size();) {
			int j = i;
			while (j < above_threshold.size() && above_threshold[j] - above_threshold[i] == j - i) {
				++j;
			}
			if (j - i >= 5) {
				consecutive_groups.push_back(std::vector<int>(above_threshold.begin() + i, above_threshold.begin() + j));
			}
			i = j;
		}

		center_indices.clear();
		for (const auto &group : consecutive_groups) {
			if (group.size() >= 5) {
				int midpoint_index = group.front() + (group.back() - group.front()) / 2;
				center_indices.push_back(midpoint_index);
			}
		}
	}

	std::vector<int> find_closest_pair(const std::vector<int> &indices, int lane_width) {
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
				int current_diff = std::abs(std::abs(indices[i] - indices[j]) - lane_width); // check for how close to optimal distance the current distance is
				if (current_diff < min_diff) {
					min_diff = current_diff; // compare current pair difference with optimal difference
					result_pair[0] = indices[i];
					result_pair[1] = indices[j];
				}
			}
		}
		return result_pair;
	}

	int combination(int n, int k) {
		if (k < 0 || k > n)
			return 0;
		if (k == 0 || k == n)
			return 1;
		k = std::min(k, n - k);
		long res = 1;
		for (int i = 1; i <= k; ++i) {
			res *= n - k + i;
			res /= i;
		}
		return res;
	}

	std::vector<double> lane_fit(std::vector<double> &x_s, std::vector<double> &y_s, int degree = 2) {
		using namespace Eigen;

		if (x_s.size() != y_s.size() || x_s.size() < 3) {
			return std::vector<double>();
		}

		const VectorXd x_orig = Map<VectorXd>(x_s.data(), x_s.size());
		const VectorXd y_orig = Map<VectorXd>(y_s.data(), y_s.size());

		// Normalization parameters
		const double x_mean = x_orig.mean();
		const double y_mean = y_orig.mean();
		const double x_std = (x_orig.array() - x_mean).matrix().norm() / sqrt(x_orig.size() - 1) + 1e-8;
		const double y_std = (y_orig.array() - y_mean).matrix().norm() / sqrt(y_orig.size() - 1) + 1e-8;

		// Standardize coordinates
		VectorXd x = (x_orig.array() - x_mean) / x_std;
		VectorXd y = (y_orig.array() - y_mean) / y_std;

		// Final fit with best degree
		MatrixXd X(x.size(), degree + 1);
		for (int i = 0; i < x.size(); ++i) {
			X(i, 0) = 1.0;
			double y_power = y[i];
			for (int j = 1; j <= degree; ++j) {
				X(i, j) = y_power;
				y_power *= y[i];
			}
		}

		VectorXd coeffs = X.colPivHouseholderQr().solve(x);

		// Convert coefficients to original space
		VectorXd c_orig = VectorXd::Zero(degree + 1);
		for (int k = 0; k <= degree; ++k) {
			double scaled_coeff = coeffs[k] * x_std / std::pow(y_std, k);
			for (int i = 0; i <= k; ++i) {
				c_orig[i] += scaled_coeff * combination(k, i) * std::pow(-y_mean, k - i);
			}
		}
		c_orig[0] += x_mean;

		return std::vector<double>(c_orig.data(), c_orig.data() + c_orig.size());
	}

	bool line_fit(const cv::Mat &binary_warped) {
		// Declare variables to be used
		static int lane_width = 350 * scale_factor; // HARD CODED LANE WIDTH
		static int n_windows = 9;	 // HARD CODED WINDOW NUMBER FOR LANE PARSING
		static int threshold = 2000; // HARD CODED THRESHOLD
		static int leftx_base = 0;
		static int rightx_base = 640 * scale_factor;
		number_of_fits = 0;
		left_fit = {0.0};
		right_fit = {0.0};

		cv::reduce(binary_warped(cv::Range(200 * scale_factor, 480 * scale_factor), cv::Range::all()) / 2, histogram, 0, cv::REDUCE_SUM, CV_32S);

		find_center_indices(histogram, threshold); // Get the center indices

		int size_indices = center_indices.size(); // Number of lanes detected

		if (size_indices == 0) { // Check to see if lanes detected, if not return
			number_of_fits = 0;
			return false;
		}

		if (size_indices == 1) {		   // If only one lane line is detected
			if (center_indices[0] < 320 * scale_factor) { // Check on which side of the car it is
				number_of_fits = 1;		   // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
				leftx_base = center_indices[0];
				rightx_base = 0;
			} else {
				number_of_fits = 3; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
				leftx_base = 0;
				rightx_base = center_indices[0];
			}
		} else {
			if (size_indices > 2) { // If more than two lane lines are detected
				std::vector<int> closest_pair = find_closest_pair(center_indices, lane_width);
				center_indices[0] = closest_pair[0]; // Initialize the start of the lane line at bottom of the screen
				center_indices[1] = closest_pair[1];
			}
			int delta = std::abs(center_indices[0] - center_indices[1]); // Check to see if the two lane lines are close enough to be the same
			if (delta < 160) {
				center_indices[0] = 0.5 * (center_indices[0] + center_indices[1]);
				if (center_indices[0] < 320 * scale_factor) {
					number_of_fits = 1; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
					leftx_base = center_indices[0];
					rightx_base = 0;
				} else {
					number_of_fits = 3; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
					leftx_base = 0;
					rightx_base = center_indices[0];
				}
			} else {
				leftx_base = center_indices[0]; // Initialize the start of the lane line at bottom of the screen
				rightx_base = center_indices[1];
				number_of_fits = 2; // Set number of fits as a reference
			}
		}

		int window_height = static_cast<int>(binary_warped.rows / n_windows); // Caclulate height of parsing windows
		// Find nonzero pixel locations
		static std::vector<cv::Point> nonzero;

		cv::findNonZero(binary_warped, nonzero); // Find nonzero values in OpenCV point format

		// Separate x and y coordinates of nonzero pixels
		std::vector<int> nonzeroy, nonzerox;
		for (size_t i = 0; i < nonzero.size(); i += 2) { // Increment index by 2
			nonzeroy.push_back(nonzero[i].y);
			nonzerox.push_back(nonzero[i].x);
		}

		// Current positions to be updated for each window
		int leftx_current = leftx_base;
		int rightx_current = rightx_base;

		// Set the width of the windows +/- margin
		int margin = 50 * scale_factor;

		// Set minimum number of pixels found to recenter window
		int minpix = 50 * scale_factor;

		// Create empty vectors to receive left and right lane pixel indices
		std::vector<int> left_lane_inds;
		std::vector<int> right_lane_inds;

		for (int window = 0; window < n_windows; ++window) {
			// Identify window boundaries in y
			int win_y_low = binary_warped.rows - (window + 1) * window_height;
			int win_y_high = binary_warped.rows - window * window_height;

			// LEFT LANE
			if (number_of_fits == 1 || number_of_fits == 2) {
				int win_xleft_low = leftx_current - margin; // Bounding boxes around the lane lines
				int win_xleft_high = leftx_current + margin;
				int sum_left = 0;
				std::vector<int> good_left_inds;
				for (size_t i = 0; i < nonzerox.size(); ++i) { // Parse through and only select pixels within the bounding boxes
					if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high && nonzerox[i] >= win_xleft_low && nonzerox[i] < win_xleft_high) {
						good_left_inds.push_back(i); // Keep pixels within the boxes
						sum_left += nonzerox[i];
					}
				}

				left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end()); // Append all good indices together

				if (good_left_inds.size() > minpix) { // Recenter mean for the next bounding box
					int mean_left = sum_left / good_left_inds.size();
					leftx_current = mean_left;
				}
			}

			// RIGHT LANE
			if (number_of_fits == 3 || number_of_fits == 2) {
				int win_xright_low = rightx_current - margin; // Bounding boxes around the lane lines
				int win_xright_high = rightx_current + margin;
				int sum_right = 0;
				std::vector<int> good_right_inds;
				for (size_t i = 0; i < nonzerox.size(); ++i) { // Parse through and only select pixels within the bounding boxes
					if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high && nonzerox[i] >= win_xright_low && nonzerox[i] < win_xright_high) {
						good_right_inds.push_back(i); // Keep pixels within the boxes
						sum_right += nonzerox[i];
					}
				}

				right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end()); // Append all good indices together

				if (good_right_inds.size() > minpix) { // Keep pixels within the boxes
					int mean_right = sum_right / good_right_inds.size();
					rightx_current = mean_right;
				}
			}
		}

		// Declare vectors to contain the pixel coordinates to fit
		std::vector<double> leftx;
		std::vector<double> lefty;
		std::vector<double> rightx;
		std::vector<double> righty;

		// int m = 3; // degree of the polynomial

		if (number_of_fits == 1 || number_of_fits == 2) {
			// Populate leftx and lefty vectors
			for (int idx : left_lane_inds) {
				leftx.push_back(nonzerox[idx]);
				lefty.push_back(nonzeroy[idx]);
			}

			left_fit = lane_fit(leftx, lefty);
            
            if (scale_factor != 1.0) {
                left_fit[0] /= scale_factor;       // a0' → a0 = a0'/scale
                left_fit[2] *= scale_factor;       // a2' → a2 = a2' * scale
            }

			// Perform polynomial fitting
			/* static alglib::real_1d_array x_left, y_left;           // Declare alglib array type */
			/* x_left.setcontent(leftx.size(), leftx.data());  // Populate X array */
			/* y_left.setcontent(lefty.size(), lefty.data());  // Populate Y array */
			/* static alglib::polynomialfitreport rep_left; */
			/* static alglib::barycentricinterpolant p_left; */
			/* alglib::polynomialfit(y_left, x_left, m, p_left, rep_left);     // Perform polynomial fit */
			/* // Convert polynomial coefficients to standard form */
			/* static alglib::real_1d_array a1; */
			/* alglib::polynomialbar2pow(p_left, a1); */
			/* left_fit.clear(); */
			/* int size = a1.length(); */
			/* left_fit.reserve(size); */
			/* for (int i = 0; i < size; ++i) {    // Iterate over to to transform */
			/*     left_fit.push_back(a1[i]); */
			/* } */
		}

		if (number_of_fits == 3 || number_of_fits == 2) {
			// Populate rightx and righty vectors
			for (int idx : right_lane_inds) {
				rightx.push_back(nonzerox[idx]);
				righty.push_back(nonzeroy[idx]);
			}

			right_fit = lane_fit(rightx, righty);

            if (scale_factor != 1.0) {
                right_fit[0] /= scale_factor;       // a0' → a0 = a0'/scale
                right_fit[2] *= scale_factor;       // a2' → a2 = a2' * scale
            }

			// Perform polynomial fitting
			/* static alglib::real_1d_array x_right, y_right;             // Declare alglib array type */
			/* x_right.setcontent(rightx.size(), rightx.data());   // Populate X array */
			/* y_right.setcontent(righty.size(), righty.data());   // Populate Y array */
			/* static alglib::polynomialfitreport rep_right; */
			/* static alglib::barycentricinterpolant p_right; */
			/* alglib::polynomialfit(y_right, x_right, m, p_right, rep_right);     // Perform polynomial fit */
			/* // Convert polynomial coefficients to standard form */
			/* static alglib::real_1d_array a3; */
			/* alglib::polynomialbar2pow(p_right, a3); */
			/* right_fit.clear(); */
			/* int size = a3.length(); */
			/* right_fit.reserve(size); */
			/* for (int i = 0; i < size; ++i) { */
			/*     right_fit.push_back(a3[i]); */
			/* } */
		}
		return true;
	}

	cv::Mat viz3(const cv::Mat &binary_warped, const cv::Mat &non_warped, const std::vector<float> waypoints, bool IPM = true) {

		// Generate y values for plotting
		std::vector<double> ploty;
		for (int i = 0; i < binary_warped.rows; ++i) {
			ploty.push_back(i);
		}

		// Create an empty image
		cv::Mat result(binary_warped.size(), CV_8UC3, cv::Scalar(0, 0, 0));

		// Update values only if they are not None
		std::vector<double> left_fitx, right_fitx;
		if (number_of_fits == 1 || number_of_fits == 2) {
			for (double y : ploty) {
				left_fitx.push_back(left_fit[0] + y * left_fit[1] + left_fit[2] * (y * y));
			}
		}
		if (number_of_fits == 3 || number_of_fits == 2) {
			for (double y : ploty) {
				right_fitx.push_back(right_fit[0] + y * right_fit[1] + right_fit[2] * (y * y));
			}
		}

		if (number_of_fits == 1 || number_of_fits == 2) {
			std::vector<cv::Point> left_points;
			for (size_t i = 0; i < left_fitx.size(); ++i) {
				left_points.push_back(cv::Point(left_fitx[i], ploty[i]));
			}
			cv::polylines(result, left_points, false, cv::Scalar(255, 255, 0), 15);
		}
		if (number_of_fits == 3 || number_of_fits == 2) {
			std::vector<cv::Point> right_points;
			for (size_t i = 0; i < right_fitx.size(); ++i) {
				right_points.push_back(cv::Point(right_fitx[i], ploty[i]));
			}
			cv::polylines(result, right_points, false, cv::Scalar(255, 255, 0), 15);
		}

		// Draw waypoints
		// for (size_t i = 0; i < y_Values.size(); ++i) {
		//     int x = static_cast<int>(waypoints[i]);
		//     int y = y_Values[i];
		//     cv::circle(result, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
		// }
		for (int i = 0; i < waypoints.size(); i += 3) {
			int x = 320 - static_cast<int>(waypoints[i + 1] / PIXEL_X_TO_METERS);
			int y = 480 - static_cast<int>(waypoints[i] / PIXEL_Y_TO_METERS);
			cv::circle(result, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
		}

		// // Draw stop line
		if (stop_line) {
			cv::line(result, cv::Point(0, stop_index), cv::Point(639, stop_index), cv::Scalar(0, 0, 255), 2);
		}
		if (IPM) {
			cv::addWeighted(result, 0.95, binary_warped, 0.3, 0, result);
		}

		if (!IPM) {
			// Apply inverse perspective transform
			cv::Mat result_ipm;
			cv::warpPerspective(result, result_ipm, invMatrix, binary_warped.size(), cv::INTER_LINEAR);
			cv::addWeighted(non_warped, 0.3, result_ipm, 0.95, 0, result);
		}

		if (stop_line) {
			cv::putText(result, "Stopline detected!", cv::Point(64, 48), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
		}

		if (cross_walk) {
			cv::putText(result, "Crosswalk detected!", cv::Point(128, 96), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
		}

		return result;
	}
};
