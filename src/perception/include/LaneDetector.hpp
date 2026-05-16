#pragma once

#include "stdafx.h"
#include "utils/Lane3.h"
#include "std_msgs/Float32.h"
#include "utils/constants.h"
#include "utils/realsense_tunables.h"
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
					std::vector<double> realsense_tf_values;
					if (!nh.getParam("/real/realsense_tf_real", realsense_tf_values)) {
							ROS_ERROR("Missing param: /real/realsense_tf_real");
							exit(1);
					}
					std::array<double, 6> realsense_tf_real;
					try {
							realsense_tf_real = VehicleTunables::to_realsense_tf_array(
									realsense_tf_values,
									"/real/realsense_tf_real"
							);
					} catch (const std::exception& exc) {
							ROS_ERROR_STREAM(exc.what());
							exit(1);
					}

					// 1) build intrinsic K
					double fx = CAMERA_PARAMS_REAL[0], fy = CAMERA_PARAMS_REAL[1],
								 cx = CAMERA_PARAMS_REAL[2], cy = CAMERA_PARAMS_REAL[3];
					cv::Mat K = (cv::Mat_<double>(3, 3) <<
							fx,  0, cx,
							 0, fy, cy,
							 0,  0,  1);
	
					// 2) build rotation R from yaw/pitch/roll (in radians)
					double yaw   = realsense_tf_real[5];
					double pitch = realsense_tf_real[4];
					double roll  = realsense_tf_real[3];
	
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
					cv::Mat C = (cv::Mat_<double>(3,1) << realsense_tf_real[0],
							realsense_tf_real[1],
							realsense_tf_real[2]);
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
	LaneDetector(ros::NodeHandle &nh, bool publish_msg) : nh(nh), showflag(false), printflag(false),
		ipm_camera(nh, true), publish_msg(publish_msg)
	{
		IPM_WIDTH = ipm_camera.resolution * ipm_camera.width_m;
		IPM_HEIGHT = ipm_camera.resolution * (ipm_camera.far_m - ipm_camera.near_m);
		METER_PER_PIXEL_X = 1 / ipm_camera.resolution;
		METER_PER_PIXEL_Y = 1 / ipm_camera.resolution;
		lane_pub = nh.advertise<utils::Lane3>("/lane", 1);
		center_offset_pub = nh.advertise<std_msgs::Float32>("/lane_center_offset", 1);
		waypoints_pub = nh.advertise<std_msgs::Float32MultiArray>("/lane_waypoints", 1);
		std::string nodeName = ros::this_node::getName();
		nh.getParam(nodeName + "/showFlag", showflag);
		nh.getParam(nodeName + "/printFlag", printflag);
		nh.getParam(nodeName + "/pub", publish);
		nh.param(nodeName + "/printDuration", printDuration, false);
		nh.getParam(nodeName + "/real", real);

		double window_margin_multiplier;
		nh.getParam("window_margin_multiplier", window_margin_multiplier);
		WINDOW_MARGIN = int(window_margin_multiplier * VehicleConstants::LANE_WHITE * ipm_camera.resolution);
		nh.getParam("window_min_pixels", WINDOW_MIN_PIXELS);
		nh.getParam("window_height", window_height);
		n_windows = int((ipm_camera.far_m - ipm_camera.near_m) / window_height);
		nh.getParam("stopline_distance_threshold", stopline_distance_threshold);
		if(!nh.getParam("show_lane", showlane)) {
			std::cerr << "show_lane parameter not found, exiting." << std::endl;
			exit(1);
		}
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

	bool publish_msg;
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

	IPMCamera ipm_camera;

	ros::NodeHandle nh;
	ros::Publisher lane_pub;
	ros::Publisher center_offset_pub;
	ros::Publisher waypoints_pub;

	bool showflag, printflag, printDuration, publish, real, showlane = false;

	// NEW LANE
	cv::Mat processed_image = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	cv::Mat ipm_color = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC3);
	cv::Mat ipm_processed = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);
	cv::Mat binary_image = cv::Mat::zeros(IMG_HEIGHT, IMG_WIDTH, CV_8UC1);

	std_msgs::Float32MultiArray waypoints_msg;
	std_msgs::MultiArrayDimension dimension;

	// LINE FIT
	struct StopLineOut
	{
			bool  found         = false;   ///< true if everything passed
			float dist_m        = -1.f;    ///< front‑bumper → stop‑line (metres)
			float angle_deg     = 0.f;     ///< yaw   (0° = perfect horizontal)
			float rmsErr_px     = 0.f;     ///< straightness metric
			float confidence    = 0.f;     ///< 0‑to‑1 quality score
			cv::Vec4f segment   = {0,0,0,0};///< (x1,y1,x2,y2)               (optional)
	};
	double stopline_dist = -1.0, stopline_yaw = -1.0;
	StopLineOut sl;
	double stopline_dist_to_front = -1.0;
	double stopline_distance_threshold;
	bool stopline = false;
	bool cross_walk = false;
	int lane_to_fit = NONE;
	VectorXd left_fit = VectorXd(4);
	VectorXd right_fit = VectorXd(4);
	cv::Mat histogram;

	StopLineOut detectStopLineColumns(const cv::Mat& bin,
                                         int   laneWidthPx,
                                         float meterPerPixelY,
                                         int   colStep      = 4,   // probe every 4 px
                                         int   yTol         = 5)   // ± px around r̂top
	{
			CV_Assert(bin.type() == CV_8UC1);

			const int rows = bin.rows, cols = bin.cols;

			// ------------------------------------------------ 1. histogram to get r̂top
			cv::Mat rowSum;
			cv::reduce(bin, rowSum, 1, cv::REDUCE_SUM, CV_32S);

			const int minRunPx = int(laneWidthPx * 0.4f);   // contiguous paint length ≥ 40 % lane
			int rHat = -1;

			auto longestRun = [&](const uchar* row)->int
			{
					int best = 0, cur = 0;
					for (int c = 0; c < cols; ++c)
							if (row[c]) { best = std::max(best, ++cur); }
							else        { cur = 0; }
					return best;
			};

			for (int r = rows - 1; r >= 0; --r)
					if (longestRun(bin.ptr<uchar>(r)) >= minRunPx)
					{ rHat = r; break; }

			StopLineOut out;
			if (rHat == -1) return out;               // nothing wide enough

			// ------------------------------------------------ 2. column probes → point cloud
			std::vector<cv::Point2f> pts; pts.reserve(cols / colStep);

			const int yMin = std::max(0,  rHat - yTol);
			const int yMax = std::min(rows - 1, rHat + yTol);

			for (int c = 0; c < cols; c += colStep)
			{
					for (int r = yMin; r <= yMax; ++r)    // scan downward only inside the band
							if (bin.at<uchar>(r, c))
							{
									pts.emplace_back(float(c), float(r));
									break;                        // next column
							}
			}
			if (pts.size() < 4) return out;           // not enough evidence

			// ------------------------------------------------ 3. RANSAC line fit (robust to strays)
			cv::Vec4f line;
			cv::fitLine(pts, line, cv::DIST_L2, 0, 0.01, 0.01); // (vx,vy,x0,y0)

			const float vx = line[0], vy = line[1];
			out.angle_deg  = -std::atan2(vy, vx) * 180.f / CV_PI;   // flip Y‑axis

			// end‑points (project extrema) – for debugging overlay
			float minProj =  1e6f, maxProj = -1e6f;
			for (auto& p : pts)
			{
					float proj = (p.x - line[2])*vx + (p.y - line[3])*vy;
					minProj = std::min(minProj, proj);
					maxProj = std::max(maxProj, proj);
			}
			cv::Point2f p1(line[2] + vx*minProj, line[3] + vy*minProj);
			cv::Point2f p2(line[2] + vx*maxProj, line[3] + vy*maxProj);
			out.segment = {p1.x, p1.y, p2.x, p2.y};

			// ------------------------------------------------ 4. distance (same as before)
			float pixelDist = float(rows - rHat);
			out.dist_m      = pixelDist * meterPerPixelY + ipm_camera.near_m;

			// ------------------------------------------------ 5. straightness & confidence
			double sumSq = 0.0;
			for (auto& p : pts)
					sumSq += std::pow((p.x - line[2])*vy - (p.y - line[3])*vx, 2);
			out.rmsErr_px = std::sqrt(sumSq / pts.size());

			const float errOk = std::exp(-out.rmsErr_px / 4.f);
			const float angOk = std::exp(-std::pow(out.angle_deg,2) / 900.f);
			const float ptsOk = std::min(1.f, float(pts.size()) / 30.f);
			out.confidence    = 0.4f*ptsOk + 0.4f*errOk + 0.2f*angOk;
			out.found         = out.confidence > 0.35f;
			return out;
	}
	StopLineOut findStopLineEx(const cv::Mat& image,
                                  int   laneWidthPx,
                                  float meterPerPixelY)
	{
			CV_Assert(image.type() == CV_8UC1);

			const int rows = image.rows;
			const int cols = image.cols;

			// ------------------------------------------------------------------ 1. Row histogram
			cv::Mat rowSum;                                   // CV_32S, rows×1
			cv::reduce(image, rowSum, 1, cv::REDUCE_SUM, CV_32S);

			const int minWhite = int(laneWidthPx * 0.75f);    // = threshold [px]

			// ------------------------------------------------------------------ 2. Locate band (contiguous wide rows)
			int bandBottom = -1, bandTop = -1;

			const int minRun = int(laneWidthPx * 0.40f);   // 40 % lane width

			auto longestRun = [&](const uchar* row, int cols)->int
			{
					int best = 0, cur = 0;
					for (int c = 0; c < cols; ++c)
					{
							if (row[c]) { ++cur; best = std::max(best, cur); }
							else        { cur = 0; }
					}
					return best;                           // length in px
			};
			for (int r = rows-1; r >= 0; --r)
			{
					if (longestRun(image.ptr<uchar>(r), cols) >= minRun)
					{
							if (bandBottom == -1) bandBottom = r;
							bandTop = r;
					}
					else if (bandBottom != -1)
							break;
			}

			StopLineOut out;
			if (bandBottom == -1)                // nothing wide enough
					return out;

			// ------------------------------------------------------------------ 3. Distance (your old maths)
			const int bandCentre = (bandTop + bandBottom) / 2;
			float pixelDist = float(rows - bandCentre);       // camera → line [px]
			out.dist_m      = pixelDist * meterPerPixelY + ipm_camera.near_m;

			// ------------------------------------------------------------------ 4. Row‑centroid cloud
			std::vector<cv::Point2f> pts;
			pts.reserve(bandBottom - bandTop + 1);

			for (int r = bandTop; r <= bandBottom; ++r)
			{
					const uchar* row = image.ptr<uchar>(r);

					int bestStart = -1, bestLen = 0, curStart = -1, curLen = 0;
					for (int c = 0; c <= cols; ++c)                  // note: <= to flush
					{
							bool white = (c < cols && row[c]);
							if (white)
							{
									if (curLen == 0) curStart = c;
									++curLen;
							}
							else if (curLen)
							{
									if (curLen > bestLen) { bestLen = curLen; bestStart = curStart; }
									curLen = 0;
							}
					}
					if (bestLen >= minRun)
					{
							float cx = bestStart + bestLen * 0.5f;
							pts.emplace_back(cx, float(r));
					}
			}

			if (pts.size() < 2)                 // should never happen, but be safe
					return out;

			// ------------------------------------------------------------------ 5. Fit a line through centroids
			cv::Vec4f ln;
			cv::fitLine(pts, ln, cv::DIST_L2, 0, 0.01, 0.01);   // (vx,vy,x0,y0)

			const float vx = ln[0], vy = ln[1];
			out.angle_deg  = std::atan2(vy, vx) * 180.f / CV_PI;

			// ------------------------------------------------------------------ 6. RMS error (straightness)
			double sumSq = 0.0;
			for (auto& p : pts)
					sumSq += std::pow( (p.x - ln[2])*vy - (p.y - ln[3])*vx, 2 );

			out.rmsErr_px = std::sqrt(sumSq / pts.size());

			// ------------------------------------------------------------------ 7. Confidence
			const float lenPx     = float(bandBottom - bandTop + 1);   // vertical span
			const float errOk     = std::exp(-out.rmsErr_px / 4.f);    // 1 → 0.6 @ 2.8 px
			const float angOk     = std::exp(-std::pow(out.angle_deg, 2) / 900.f);
			const float lenOk     = std::min(1.f, lenPx / 6.f);        // good if ≥6 rows

			out.confidence = 0.4f*lenOk + 0.4f*errOk + 0.2f*angOk;
			out.found      = out.confidence > 0.35f;

			// ------------------------------------------------------------------ 8. Optional: segment for drawing
			// project extrema of pts onto the line to get two endpoints
			float minProj =  1e6f, maxProj = -1e6f;
			for (auto& p : pts)
			{
					float proj = (p.x - ln[2])*vx + (p.y - ln[3])*vy;
					minProj = std::min(minProj, proj);
					maxProj = std::max(maxProj, proj);
			}
			cv::Point2f p1(ln[2] + vx*minProj, ln[3] + vy*minProj);
			cv::Point2f p2(ln[2] + vx*maxProj, ln[3] + vy*maxProj);
			out.segment = { p1.x, p1.y, p2.x, p2.y };

			return out;
	}

	bool good_fit = false;
	utils::Lane3 publish_lane(const cv::Mat &image) {
		utils::Lane3 lane_msg;
		if (image.empty()) {
			ROS_WARN("empty image received in lane detector");
			return {};
		}
		auto start = high_resolution_clock::now();
		if (true) {
			preprocess(image, processed_image);
			if (!ipm_camera.getIPM(processed_image, ipm_processed)) return {};

			// ipm_camera.getIPM(image, ipm_color);
			// cv::imshow("ipm color", ipm_color);
			
			// cv::imshow("processed image", processed_image);
			// cv::imshow("processed ipm", ipm_processed);
			// cv::waitKey(1);
			// return;

			// StopLineOut sl = detectStopLineColumns(ipm_processed,
			// 																				LANE_WIDTH_PIXEL,
			// 																				METER_PER_PIXEL_Y);

			stopline_dist = find_stopline(ipm_processed);
			sl = findStopLineEx(ipm_processed,
                                LANE_WIDTH_PIXEL,
                                METER_PER_PIXEL_Y);
			
			if (sl.found && sl.confidence > 0.01f)
			{
					stopline_dist = sl.dist_m;
					stopline_yaw = sl.angle_deg;
			}
			double center_offset = find_lanes(ipm_processed);
			lane_msg.lane_center_offset = center_offset;
			lane_msg.stopline_dist = stopline_dist;
			lane_msg.header.stamp = ros::Time::now();
			good_fit = line_fit(ipm_processed);
			std::vector<float> wpts;
			if(good_fit) {
				wpts = get_waypoints(left_fit, right_fit, 40, 0.032);
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

				for (int i = 0; i < wpts.size(); i++) {
					lane_msg.lane_waypoints.push_back(wpts[i]);
				}
				for (int i =0; i < left_waypoints.size(); i++) {
					lane_msg.left_waypoints.push_back(left_waypoints[i]);
				}
				for (int i =0; i < right_waypoints.size(); i++) {
					lane_msg.right_waypoints.push_back(right_waypoints[i]);
				}
				lane_msg.straight_lane = is_straight;
				lane_msg.straight_lane_angle = waypoints_angle;
				lane_msg.good_left = good_left;
				lane_msg.good_right = good_right;
				lane_msg.near_m = ipm_camera.near_m;
			} else {
				is_straight = false;
			}
			
			if (publish_msg) lane_pub.publish(lane_msg);

			if (showflag || showlane) {
				if (!ipm_camera.getIPM(image, ipm_color))
					return lane_msg;
				viz3(ipm_color, image, wpts, true);
				if (showflag) {
					cv::imshow("viz3", viz_img);
					cv::waitKey(1);
				}
			}
		}
		if (printDuration) {
			auto stop = high_resolution_clock::now();
			auto duration = duration_cast<microseconds>(stop - start);
			ROS_INFO("lane duration: %ld", duration.count());
		}
		return lane_msg;
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

	bool is_straight = false;
	double waypoints_angle = 0.0;
	std::vector<float> flat;
	std::vector<cv::Point2f> world_waypoints;
	std::vector<float> left_waypoints;
	std::vector<float> right_waypoints;
	std::vector<float> get_waypoints(const VectorXd &left_fit,
                                               const VectorXd &right_fit,
                                               int   num_waypoints,
                                               double density)
	{
			constexpr double kBEND_TH = 1.02;                //  ≈2 % longer than chord
			constexpr double kYAW_TH  = 2.0 * M_PI / 180.0;  // 2 deg in radians

			world_waypoints.clear();
			left_waypoints.clear();
			right_waypoints.clear();
			world_waypoints.reserve(num_waypoints);
			left_waypoints.reserve(num_waypoints);
			right_waypoints.reserve(num_waypoints);

			// ---------- 1. build way‑points in the body‑fixed (x = forward) frame ----
			double current_y_meter = 0.0;

			for (int i = 0; i < num_waypoints; ++i)
			{
					double y_pixel = IPM_HEIGHT - meter_to_pixel_y(current_y_meter);

					double left_x   = evaluate_poly(y_pixel, left_fit);
					double right_x  = evaluate_poly(y_pixel, right_fit);
					double center_x = 0.5 * (left_x + right_x);
					if (i == 0 && good_left && good_right) {
						double estimated_lane_width = std::abs(left_x - right_x);
						if (std::abs(estimated_lane_width-LANE_WIDTH_PIXEL) > 0.25 * LANE_WIDTH_PIXEL) {
							// std::cout << "Inconsistent lane width detected, ratio: " << std::abs(estimated_lane_width-LANE_WIDTH_PIXEL)/LANE_WIDTH_PIXEL << ", LANE_WIDTH_PIXEL: " << LANE_WIDTH_PIXEL << ", estimated: " << estimated_lane_width << std::endl;
							good_left = false;
							good_right = false;
							return {};
						}
					}
					if (!good_left) {
						center_x = right_x - LANE_WIDTH_PIXEL / 2.0;
					} else if (!good_right) {
						center_x = left_x + LANE_WIDTH_PIXEL / 2.0;
					}

					double world_x  = (IPM_WIDTH - center_x) * METER_PER_PIXEL_X - ipm_camera.width_m / 2.0;
					double world_x_left	= (IPM_WIDTH - left_x) * METER_PER_PIXEL_X - ipm_camera.width_m / 2.0;
					double world_x_right = (IPM_WIDTH - right_x) * METER_PER_PIXEL_X - ipm_camera.width_m / 2.0;
					world_waypoints.emplace_back(static_cast<float>(current_y_meter),
																			static_cast<float>(world_x));   // (x = fwd, y = left)

					
					left_waypoints.emplace_back(static_cast<float>(world_x_left));
					right_waypoints.emplace_back(static_cast<float>(world_x_right));
					// advance y by arc‑length “density” metres along the centre line
					double left_dxdy   = get_derivative(y_pixel, left_fit);
					double right_dxdy  = get_derivative(y_pixel, right_fit);
					double centre_dxdy = 0.5 * (left_dxdy + right_dxdy);

					double centre_dxdy_world = centre_dxdy * (METER_PER_PIXEL_X / METER_PER_PIXEL_Y);
					double dy = density / std::sqrt(1.0 + centre_dxdy_world * centre_dxdy_world);
					current_y_meter += dy;
			}

			// shift origin from camera to vehicle CG
			// for (auto &wp : world_waypoints) wp.x += VehicleConstants::REALSENSE_TF[0];

			// ---------- 2. compute yaw (heading) for each segment --------------------
			std::vector<float> yaw_values;
			yaw_values.reserve(world_waypoints.size());

			for (size_t i = 0; i + 1 < world_waypoints.size(); ++i)
			{
					float dx = world_waypoints[i + 1].x - world_waypoints[i].x;
					float dy = world_waypoints[i + 1].y - world_waypoints[i].y;
					yaw_values.push_back(std::atan2(dy, dx));        // rad
			}
			yaw_values.push_back(!yaw_values.empty() ? yaw_values.back() : 0.0f);

			// ---------- 3. pack (x,y,yaw) for the planner/MCU ------------------------
			
			flat.reserve(world_waypoints.size() * 3);
			flat.clear();
			for (size_t i = 0; i < world_waypoints.size(); ++i)
			{
					flat.push_back(world_waypoints[i].x);
					flat.push_back(world_waypoints[i].y);
					flat.push_back(yaw_values[i]);
			}

			// ---------- 4. STRAIGHTNESS CHECK (ROI == whole way‑point list) ----------
			double path_len = 0.0;
			for (size_t i = 0; i + 1 < world_waypoints.size(); ++i)
			{
					double dx = world_waypoints[i + 1].x - world_waypoints[i].x;
					double dy = world_waypoints[i + 1].y - world_waypoints[i].y;
					path_len += std::hypot(dx, dy);
			}

			double chord_len = std::hypot(world_waypoints.back().x - world_waypoints.front().x,
																		world_waypoints.back().y - world_waypoints.front().y);

			double bend_ratio = (chord_len > 1e-6) ? path_len / chord_len : 1.0;

			auto wrap = [](double a) { return std::atan2(std::sin(a), std::cos(a)); };
			double heading_delta = std::fabs(wrap(yaw_values.front() -
																						yaw_values[yaw_values.size() - 1]));

			// ---------- 5. store results in the class --------------------------------
			is_straight = (bend_ratio < kBEND_TH) && (heading_delta < kYAW_TH);

			double yaw_sum = 0.0;
			for (float y : yaw_values) yaw_sum += y;
			waypoints_angle = yaw_sum / static_cast<double>(yaw_values.size());  // rad

			return flat;
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
			stopline_dist = pixel_to_meter_y(pixel_value) + ipm_camera.near_m; // distance from line to car center
			stopline_dist_to_front = stopline_dist;
			stopline_dist_to_front -= VehicleConstants::CAR_LENGTH / 2;
			if (stopline_dist_to_front > 0.0) {
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

	bool find_lanes_success = false;
	cv::Rect roiRect;
	double offset_from_center;
	double find_lanes(const cv::Mat &binaryIPM)
	{
			lane_indices.clear();
			if (binaryIPM.empty()) return -1.0;

			// ───────────────────────── Region of Interest ────────────────────────────
			constexpr float ROI_FRACTION   = 0.10f; // analyse only bottom 10 % of image
			constexpr float FRACTION_FROM_BOT = 0.06f; // start 6 % above bottom edge

			const int H = binaryIPM.rows, W = binaryIPM.cols;
			const int y0     = static_cast<int>((1.0f - ROI_FRACTION - FRACTION_FROM_BOT) * H);
			const int height = static_cast<int>(ROI_FRACTION * H);
			const int width  = static_cast<int>(0.9f * W);               // crop 5 % per side

			roiRect = cv::Rect(static_cast<int>((W - width) / 2), y0, width, height);
			const cv::Mat  roi = binaryIPM(roiRect);

			// ───────────────────── Histogram & peak extraction ───────────────────────
			cv::Mat hist;
			cv::reduce(roi, hist, 0, cv::REDUCE_SUM, CV_32S);
			extract_lanes(hist);                                    // fills lane_indices (relative to ROI)

			// Bring indices into full‑image coordinate system so drawings & maths align
			for (int &x : lane_indices) x += roiRect.x;

			// ─────────────────────────── Visualisation ───────────────────────────────
			// cv::Mat vis;                                           // colour copy for drawing
			// if (true)
			// {
			// 		cv::cvtColor(binaryIPM, vis, cv::COLOR_GRAY2BGR);   // → BGR so we can colour

			// 		cv::rectangle(vis, roiRect, cv::Scalar(255, 0, 0), 2);
			// 		for (int x : lane_indices)
			// 		{
			// 				cv::line(vis,
			// 								cv::Point(x, roiRect.y),
			// 								cv::Point(x, roiRect.y + roiRect.height),
			// 								cv::Scalar(0, 0, 255), 3);
			// 		}
			// }

			offset_from_center = -1.0;                       // default: fail

			find_lanes_success = false; // reset success flag
			if (lane_indices.size() == 2 && lane_indices[0] > 0 && lane_indices[1] > 0)
			{
					const int diff = lane_indices[1] - lane_indices[0];
					if (diff > 0 && std::abs(diff - LANE_WIDTH_PIXEL) < 0.15 * LANE_WIDTH_PIXEL)
					{
							find_lanes_success = true;
							offset_from_center = -(lane_indices[0] + lane_indices[1] - W) / 2.0 * METER_PER_PIXEL_X;

							// if (true)
							// {
							// 		const int midX = (lane_indices[0] + lane_indices[1]) / 2;
							// 		const int midY = roiRect.y + roiRect.height / 2;
							// 		cv::circle(vis, cv::Point(midX, midY), 8, cv::Scalar(0, 255, 0), -1);
							// 		char buf[64];
							// 		std::snprintf(buf, sizeof(buf), "offset: %.2fm", offset_from_center);
							// 		cv::putText(vis, buf, cv::Point(midX + 10, midY - 10),
							// 								cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
							// }
					}
			}

			// if (true)
			// {
			// 		cv::imshow("Lane Visualisation", vis);
			// 		cv::waitKey(1);
			// }

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

	bool good_left = false;
	bool good_right = false;
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

		// DISPLAY
		// cv::Mat out_img = cv::Mat::zeros(binary_warped.size(), CV_8UC3);

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
				// cv::rectangle(out_img, cv::Point(win_xleft_low, win_y_low),
        //           cv::Point(win_xleft_high, win_y_high),
        //           cv::Scalar(0, 255, 0), 2);

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
						// cv::circle(out_img, cv::Point(mean_left, (win_y_low + win_y_high) / 2), 10, cv::Scalar(0, 255, 255), -1);
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
				// cv::rectangle(out_img, cv::Point(win_xright_low, win_y_low),
				// 					cv::Point(win_xright_high, win_y_high),
				// 					cv::Scalar(0, 255, 0), 2);  // Green for right lane window

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
						// cv::circle(out_img, cv::Point(mean_right, (win_y_low + win_y_high) / 2), 10, cv::Scalar(0, 255, 255), -1);
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
		// for (int i = 0; i < left_lane_inds.size(); ++i) {
		// 	cv::circle(out_img, cv::Point(nonzerox[left_lane_inds[i]], nonzeroy[left_lane_inds[i]]), 2, cv::Scalar(0, 0, 255), -1);
		// }
		// for (int i = 0; i < right_lane_inds.size(); ++i) {
		// 	cv::circle(out_img, cv::Point(nonzerox[right_lane_inds[i]], nonzeroy[right_lane_inds[i]]), 2, cv::Scalar(255, 0, 0), -1);
		// }
		// cv::imshow("Detected Pixels", out_img);
		// cv::waitKey(1);

		// std::cout << "num_left_windows: " << num_left_windows << ", num_right_windows: " << num_right_windows << std::endl;
		// Declare vectors to contain the pixel coordinates to fit
		VectorXd leftx;
		VectorXd lefty;
		VectorXd rightx;
		VectorXd righty;

		static constexpr int num_window_thresh = 10;
		good_left  = false;
		good_right = false;

		/* -------- 1. fit the sides that have enough windows ---------------- */
		if (lane_to_fit == LEFT || lane_to_fit == BOTH) {
				if (num_left_windows >= num_window_thresh) {
						fit_points(leftx, lefty, left_lane_inds,
											nonzeroy, nonzerox, left_fit);
						
						if (lane_to_fit == LEFT) {
							right_fit = left_fit;
							right_fit(0) += LANE_WIDTH_PIXEL;
						}
						good_left = true;
				}
		}

		if (lane_to_fit == RIGHT || lane_to_fit == BOTH) {
				if (num_right_windows >= num_window_thresh) {
						fit_points(rightx, righty, right_lane_inds,
											nonzeroy, nonzerox, right_fit);
						good_right = true;
						if (lane_to_fit == RIGHT) {
							left_fit = right_fit;
							left_fit(0) -= LANE_WIDTH_PIXEL;
						}
				}
		}

		/* -------- 2. insist on seeing **both** real sides ------------------ */
		if (!good_left && !good_right) {
				return false;                 // discard this frame immediately
		}

		/* 2.  Lane‑width and curvature consistency (only if both sides exist) */
		// bool width_ok = true, curv_ok = true;
		// if (good_left || good_right)
		// {
		// 		// sample at three y‑locations in the lower half
		// 		for (int y = img_height-1; y > img_height/2; y -= img_height/6) {
		// 				double lx = evaluate_poly(y, left_fit);
		// 				double rx = evaluate_poly(y, right_fit);
		// 				double width = std::abs(rx - lx);
		// 				if (std::fabs(width - LANE_WIDTH_PIXEL) > 15.0) { width_ok = false; break; }
		// 		}
		// 		// curvature κ = |y''| / (1+y'^2)^(3/2); here use mid‑ROI point
		// 		auto curvature = [](const VectorXd& p, double y){
		// 				double dy  = 3*p(0)*y*y + 2*p(1)*y + p(2);
		// 				double d2y = 6*p(0)*y   + 2*p(1);
		// 				return std::abs(d2y) / std::pow(1 + dy*dy, 1.5);
		// 		};
		// 		double y_mid = img_height*0.6;
		// 		double kL = curvature(left_fit,  y_mid);
		// 		double kR = curvature(right_fit, y_mid);
		// 		curv_ok = std::fabs(kL - kR) < 0.001;          // px‑1
		// }

		// bool fits_good = false;
		// if (good_left || good_right)        fits_good = width_ok && curv_ok;

		// if (!fits_good)       return false;   // discard this frame
				
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

	cv::Mat viz_img;
	void viz3(const cv::Mat &binary_warped, const cv::Mat &non_warped, const std::vector<float> waypoints, bool IPM = true) {
		const int img_height = binary_warped.rows;
		const int img_width = binary_warped.cols;
		cv::Mat result(binary_warped.size(), CV_8UC3, cv::Scalar(0, 0, 0));

		if (good_fit) {
			std::vector<double> ploty;
			for (int i = 0; i < binary_warped.rows; ++i) {
				ploty.push_back(i);
			}
			std::vector<double> left_fitx, right_fitx;
			if (lane_to_fit == LEFT || lane_to_fit == BOTH) {
				for (double y : ploty) {
					left_fitx.push_back(evaluate_poly(y, left_fit));
				}
				std::vector<cv::Point> left_points;
				for (size_t i = 0; i < left_fitx.size(); ++i) {
					left_points.push_back(cv::Point(left_fitx[i], ploty[i]));
				}
				auto left_color = cv::Scalar(255, 255, 0);
				// Orange if left lane is not good
				if (!good_left) left_color = cv::Scalar(0, 165, 255);
				cv::polylines(result, left_points, false, left_color, 3);
			}
			if (lane_to_fit == RIGHT || lane_to_fit == BOTH) {
				for (double y : ploty) {
					right_fitx.push_back(evaluate_poly(y, right_fit));
				}
				std::vector<cv::Point> right_points;
				for (size_t i = 0; i < right_fitx.size(); ++i) {
					right_points.push_back(cv::Point(right_fitx[i], ploty[i]));
				}
				auto right_color = cv::Scalar(255, 255, 0);
				// Orange if right lane is not good
				if (!good_right) right_color = cv::Scalar(0, 165, 255);
				cv::polylines(result, right_points, false, right_color, 3);
			}
			for (int i = 0; i < waypoints.size(); i += 3) {
				int x = img_width/2 - static_cast<int>((waypoints[i + 1]) / METER_PER_PIXEL_X);
				// int y = img_height - static_cast<int>(waypoints[i] / METER_PER_PIXEL_Y);
				int y = img_height - static_cast<int>(meter_to_pixel_y(waypoints[i]));
				cv::circle(result, cv::Point(x, y), 2, cv::Scalar(0, 0, 255), -1);
			}
		}

		// // Draw stop line
		if (stopline_dist > 0) {
			double stopline_y = img_height - meter_to_pixel_y(stopline_dist - ipm_camera.near_m);
			cv::line(result, cv::Point(0, stopline_y), cv::Point(img_width, stopline_y), cv::Scalar(0, 255, 0), 2);
			// if (sl.found && std::abs(stopline_yaw) > 85.0) {
			if (sl.found) {
				auto color = cv::Scalar(255, 0, 255);
				int thickness = 2;
				if (std::abs(stopline_yaw) > 85.0) {
					color = cv::Scalar(255, 255, 0);
					thickness = 4;
				}
				cv::line(result,
					{ int(sl.segment[0]), int(sl.segment[1]) },
					{ int(sl.segment[2]), int(sl.segment[3]) },
					color, thickness);
			}
		}
		if (IPM) {
			cv::addWeighted(result, 0.95, binary_warped, 0.3, 0, result);
		}

		cv::resize(result, result, cv::Size(img_width, img_height), 0, 0, cv::INTER_CUBIC);

		if (stopline_dist > 0) {
			cv::putText(result, "stop:" + std::to_string(stopline_dist) + " m", cv::Point(0, 48), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
			// cv::putText(result, "2front:" + std::to_string(stopline_dist_to_front) + " m", cv::Point(0, 48*4), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
		}

		cv::rectangle(result, roiRect, cv::Scalar(255, 0, 0), 2);
		for (int x : lane_indices)
		{
				cv::line(result,
								cv::Point(x, roiRect.y),
								cv::Point(x, roiRect.y + roiRect.height),
								cv::Scalar(0, 0, 255), 3);
		}
		if (find_lanes_success)
		{
				const int midX = (lane_indices[0] + lane_indices[1]) / 2;
				const int midY = roiRect.y + roiRect.height / 2;
				cv::circle(result, cv::Point(midX, midY), 8, cv::Scalar(0, 255, 0), -1);
				char buf[64];
				std::snprintf(buf, sizeof(buf), "offset: %.2fm", offset_from_center);
				cv::putText(result, buf, cv::Point(midX + 10, midY - 10),
										cv::FONT_HERSHEY_SIMPLEX, 0.6, cv::Scalar(0, 255, 0), 2);
		}

		{
        double angle_deg = waypoints_angle * 180.0 / CV_PI;

        // Decide colour: green if straight, red otherwise
        const cv::Scalar txt_col = is_straight ? cv::Scalar(0,255,0)
                                               : cv::Scalar(0,  0,255);

        // 1.  Text label, e.g.  “STRAIGHT  (‑1.3°)”
        std::ostringstream oss;
        oss << (is_straight ? "STRAIGHT  (" : "CURVED   (")
            << std::fixed << std::setprecision(1) << angle_deg << "deg)";
        cv::putText(result, oss.str(), cv::Point(10, 90),        // px position
                    cv::FONT_HERSHEY_SIMPLEX, 0.6, txt_col, 2, cv::LINE_AA);

        // 2.  Little arrow at the bottom centre showing lane heading
        const int arrow_len = 70;                                // pixels
        cv::Point base(img_width / 2, img_height - 20);          // tail of arrow
        double ang = -waypoints_angle;                           // screen Y axis points down
        cv::Point tip(base.x + int( arrow_len * std::sin(ang)),
                      base.y - int( arrow_len * std::cos(ang)));
        cv::arrowedLine(result, base, tip, txt_col, 3, cv::LINE_AA, 0, 0.25);
    }
		viz_img = result.clone();
	}
};
