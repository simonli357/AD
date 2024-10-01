#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <chrono>
#include "utils/Lane.h"
#include <mutex>
#include <vector>
#include <algorithm>
#include <cmath>
#include <tuple>
#include "interpolation.h"
#include "stdafx.h"
#include <iostream>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/MultiArrayDimension.h>

using namespace std::chrono;

class LaneDetector {
public:
    LaneDetector(ros::NodeHandle& nh) : nh(nh), showflag(false), printflag(false), previous_center(320)
    {
        // image_sub = it.subscribe("camera/color/image_raw", 1, &LaneDetector::imageCallback, this);
        // image_pub = it.advertise("/automobile/image_modified", 1);
        lane_pub = nh.advertise<utils::Lane>("/lane", 1);
        waypoints_pub = nh.advertise<std_msgs::Float32MultiArray>("/waypoints", 1);
        image = cv::Mat::zeros(480, 640, CV_8UC1);
        stopline = false;
        dotted = false;
        std::string nodeName = ros::this_node::getName();
        nh.getParam(nodeName+"/showFlag", showflag);
        nh.getParam(nodeName+"/printFlag", printflag);
        nh.getParam(nodeName+"/pub", publish);
        nh.param(nodeName+"/printDuration", printDuration, false);
        nh.param(nodeName+"/newlane", newlane, false);
        // ros::Rate rate(40); 
        // while (ros::ok()) {
        //     ros::spinOnce();
        //     rate.sleep();
        // }
    }

    // private:
    ros::NodeHandle nh;
    // image_transport::ImageTransport it;
    // image_transport::Subscriber image_sub;
    // image_transport::Publisher image_pub;
    ros::Publisher lane_pub;
    ros::Publisher waypoints_pub;
    utils::Lane lane_msg;
    double num_iterations = 1;
    double previous_center;
    double total;
    cv::Mat maskh, masks, image, maskd;
    bool stopline, dotted, publish;
    int h = 480, w = 640;
    std::vector<int> lanes;
    
    double minVal, maxVal;
    cv::Point minLoc, maxLoc;
    cv::Mat img_gray;
    cv::Mat img_roi;
    cv::Mat thresh;
    cv::Mat hist;
    cv::Mat img_rois;
    double threshold_value_stop;
    cv::Mat threshs;
    cv::Mat hists;
    bool showflag, printflag, newlane, printDuration;
    std::mutex mutex;

    // NEW LANE
    std::vector<int> y_Values = {475,450,420,400,350,300};
    cv::Mat image_cont_1;
    cv::Mat image_cont_2;
    cv::Mat image_cont_3;
    cv::Mat grayscale_image;
    cv::Mat horistogram;
    std::tuple<int,std::vector<double>, std::vector<double>, bool, int, bool> ret;
    std::vector<double> waypoints;
    int threshold_2 = 2000;

     // VECTOR CONTAINERS
    std::vector<double> right_1;
    std::vector<double> left_1;
    std_msgs::Float32MultiArray waypoints_msg;
    std_msgs::MultiArrayDimension dimension;

    // STATIC VARIABLES
    double height = 0.16;
    double roll = 0.15;
    double depthy;
    double laneMaxVal;
    double laneMinVal;
    // LINE FIT
    int lane_width = 350;        // HARD CODED LANE WIDTH
    int n_windows = 9;           // HARD CODED WINDOW NUMBER FOR LANE PARSING
    int threshold = 2000;       // HARD CODED THRESHOLD
    int leftx_base = 0;
    int rightx_base = 640;
    int number_of_fits = 0;
    bool stop_line = false;
    int stop_index = 0;
    bool cross_walk = false;
    int width = 370;
    int stop_loc = -1;
    cv::Mat map1, map2;
    bool maps_initialized = false;

    // Declare CAMERA_PARAMS as a constant global variable
    const std::map<std::string, double> CAMERA_PARAMS = {
        {"fx", 554.3826904296875},
        {"fy", 554.3826904296875},
        {"cx", 320},
        {"cy", 240}
    };

    // Define initial coordinates of input image as a constant global variable
    const cv::Mat initial = (cv::Mat_<float>(4, 2) <<
        0, 300,
        640, 300,
        0, 480,
        640, 480
    );

    // Define where the initial coordinates will end up on the final image as a constant global variable
    const cv::Mat final = (cv::Mat_<float>(4, 2) <<
        0, 0,
        640, 0,
        0, 480,
        640, 480
    );

    // Compute the transformation matrix
    cv::Mat transMatrix = cv::getPerspectiveTransform(initial, final);
    cv::Mat invMatrix = cv::getPerspectiveTransform(final, initial);

    // Define camera matrix for accurate IPM transform as a constant global variable
    const cv::Mat cameraMatrix = (cv::Mat_<double>(3, 3) <<
        CAMERA_PARAMS.at("fx"), 0, CAMERA_PARAMS.at("cx"),
        0, CAMERA_PARAMS.at("fy"), CAMERA_PARAMS.at("cy"),
        0, 0, 1
    );

    // Define distortion coefficients as an empty constant global variable
    const cv::Mat distCoeff = cv::Mat();

    void initializeMaps(const cv::Mat& cameraMatrix, const cv::Mat& distCoeff, const cv::Mat& transMatrix, const cv::Size& size) {
        cv::initUndistortRectifyMap(cameraMatrix, distCoeff, cv::Mat(),
                                    cameraMatrix, size, CV_16SC2, map1, map2);
        maps_initialized = true;
    }

    void publish_lane(const cv::Mat& image) {
        if(image.empty()) {
            ROS_WARN("empty image received in lane detector");
            return;
        }
        auto start = high_resolution_clock::now();
        if (newlane) {
            if (!maps_initialized) {
                initializeMaps(cameraMatrix, distCoeff, transMatrix, image.size());
            }
            static cv::Mat grayscale_image = cv::Mat::zeros(480, 640, CV_8UC1);
            static cv::Mat ipm_color = cv::Mat::zeros(480, 640, CV_8UC3);
            static cv::Mat ipm_image = cv::Mat::zeros(480, 640, CV_8UC1);
            static cv::Mat binary_image = cv::Mat::zeros(480, 640, CV_8UC1);
            cv::cvtColor(image, grayscale_image, cv::COLOR_BGR2GRAY);
            // ROS_INFO("Image converted to cv::Mat");
            auto start = high_resolution_clock::now();
            if(!getIPM(image, ipm_color)) return;
            auto start1 = high_resolution_clock::now();
            if(!getIPM(grayscale_image, ipm_image)) return;
            auto start2 = high_resolution_clock::now();
            getLanes(ipm_image, binary_image);
            auto start3 = high_resolution_clock::now();
            ret = line_fit_2(binary_image);
            auto start4 = high_resolution_clock::now();
            // printTuple(ret);
            static std::vector<double> waypoints;
            waypoints = getWaypoints(ret,y_Values);
            auto start5 = high_resolution_clock::now();
            static std::vector<double> right_1;
            right_1 = std::get<2>(ret);
            static std::vector<double> left_1;
            left_1  = std::get<1>(ret);

            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            auto duration1 = duration_cast<microseconds>(start1 - start);
            auto duration2 = duration_cast<microseconds>(start2 - start1);
            auto duration3 = duration_cast<microseconds>(start3 - start2);
            auto duration4 = duration_cast<microseconds>(start4 - start3);
            auto duration5 = duration_cast<microseconds>(start5 - start4);
            ROS_INFO("duration: %ld", duration.count());
            ROS_INFO("duration1: %ld", duration1.count());
            ROS_INFO("duration2: %ld", duration2.count());
            ROS_INFO("duration3: %ld", duration3.count());
            ROS_INFO("duration4: %ld", duration4.count());
            ROS_INFO("duration5: %ld", duration5.count());
            
            // plot_polynomial(right_1, left_1, waypoints, y_Values);
            // std_msgs::Float32MultiArray waypoints_msg;
            // std_msgs::MultiArrayDimension dimension;
            // Set dimension label and size
            dimension.label = "#ofwaypoints";
            dimension.size = 2;
            waypoints_msg.layout.dim.push_back(dimension);

            // Populate data array with waypoints
            waypoints_msg.data.push_back(waypoints[5]);
            waypoints_msg.data.push_back(stop_loc);
            // waypoints_msg.data.push_back(-wp[0]);
            // waypoints_msg.data.push_back(wp[1]);
            // waypoints_msg.data.push_back(-wp[1]);
            // waypoints_msg.data.push_back(wp[2]);
            // waypoints_msg.data.push_back(-wp[2]);
            // waypoints_msg.data.push_back(wp[3]);
            // waypoints_msg.data.push_back(-wp[3]);
            // waypoints_msg.data.push_back(wp[4]);
            // waypoints_msg.data.push_back(-wp[4]);
            // waypoints_msg.data.push_back(wp[5]);
            // waypoints_msg.data.push_back(-wp[5]);

            // Publish the message
            waypoints_pub.publish(waypoints_msg);
            lane_msg.center = waypoints[5];
            lane_msg.stopline = stop_loc;
            lane_msg.header.stamp = ros::Time::now();
            lane_pub.publish(lane_msg);

            if(showflag) {
                cv::Mat gyu_img = viz3(ipm_color,image, ret, waypoints,y_Values, true);
                cv::imshow("Binary Image", gyu_img);
                cv::waitKey(1);
            }
        } else {
            double center = optimized_histogram(image, showflag, printflag);
            lane_msg.center = center;
            lane_msg.stopline = stopline;
            lane_msg.header.stamp = ros::Time::now();
            lane_pub.publish(lane_msg);
        }
        if (printDuration) {
            auto stop = high_resolution_clock::now();
            auto duration = duration_cast<microseconds>(stop - start);
            ROS_INFO("duration: %ld", duration.count());
        }
    }

    // std::vector<int> extract_lanes(const cv::Mat& hist_data) {
    void extract_lanes(const cv::Mat& hist_data) {
        // std::vector<int> lane_indices;
        lanes.clear();
        int previous_value = 0;
        for (int idx = 0; idx < hist_data.cols; ++idx) {
            int value = hist_data.at<int>(0, idx);
            if (value >= 1500 && previous_value == 0) {
                lanes.push_back(idx);
                previous_value = 255;
            } else if (value == 0 && previous_value == 255) {
                lanes.push_back(idx);
                previous_value = 0;
            }
        }
        if (lanes.size() % 2 == 1) {
            lanes.push_back(640 - 1);
        }
        // return lane_indices;
    }

    double optimized_histogram(const cv::Mat& image, bool show = false, bool print = false) {
        stopline = false;
        cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);

        // apply maskh
        img_roi = img_gray(cv::Rect(0, 384, 640, 96));
        // cv::imshow("L", img_roi);
        // cv::waitKey(1);
        cv::minMaxLoc(img_roi, &minVal, &maxVal, &minLoc, &maxLoc);
        double threshold_value = std::min(std::max(maxVal - 55.0, 30.0), 200.0);
        cv::threshold(img_roi, thresh, threshold_value, 255, cv::THRESH_BINARY);
        hist = cv::Mat::zeros(1, w, CV_32SC1);
        cv::reduce(thresh, hist, 0, cv::REDUCE_SUM, CV_32S);

        // apply masks
        // img_rois = img_gray(cv::Range(300, 340), cv::Range::all());
        // // cv::imshow("S", img_rois);
        // // cv::waitKey(1);
        // cv::minMaxLoc(img_roi, &minVal, &maxVal, &minLoc, &maxLoc); // Use img_roi or img_rois depending on your requirements
        // threshold_value_stop = std::min(std::max(maxVal - 65.0, 30.0), 200.0);
        
        // cv::threshold(img_rois, threshs, threshold_value_stop, 255, cv::THRESH_BINARY);
        // hists = cv::Mat::zeros(1, w, CV_32SC1);
        // cv::reduce(threshs, hists, 0, cv::REDUCE_SUM, CV_32S);

        // std::vector<int> stop_lanes = extract_lanes(hists);
        // for (size_t i = 0; i < stop_lanes.size() / 2; ++i) {
        //     if (abs(stop_lanes[2 * i] - stop_lanes[2 * i + 1]) > 370 && threshold_value > 30) {
        //         stopline = true;
        //         if (!show) return w / 2.0;
        //     }
        // }

        // std::vector<int> lanes = extract_lanes(hist);
        extract_lanes(hist);
        std::vector<double> centers;
        for (size_t i = 0; i < lanes.size() / 2; ++i) {
            if (abs(lanes[2 * i] - lanes[2 * i + 1])>350 && threshold_value>50){
                stopline = true;
                if (!show) return w / 2.0;
            }
            if (3 < abs(lanes[2 * i] - lanes[2 * i + 1])) {
                centers.push_back((lanes[2 * i] + lanes[2 * i + 1]) / 2.0);
            }
        }
        double center;
        if (centers.empty()) {
            center = w / 2.0;
        } else if (centers.size() == 1) {
            center = (centers[0] > (w / 2.0)) ? (centers[0] - 0) / 2 : (centers[0] * 2 + w) / 2;
        } else if (abs(centers[0] - centers.back()) < 200) {
            center = ((centers[0] + centers.back()) > w) ? ((centers[0] + centers.back()) / 2 + 0) / 2.0 : ((centers[0] + centers.back()) + w) / 2;
        } else {
            center = (centers[0] + centers.back()) / 2;
        }

        // if(std::abs(center - previous_center) > 250) {
        //     center = previous_center;
        // }
        // if (std::abs(center - 320) < 1) {
        //     double temp = center;
        //     center = previous_center;
        //     previous_center = temp;
        // } else {
        //     previous_center = center;
        // }

        if (show) {
            // Create the new cv::Mat object and initialize it with zeros
            cv::Mat padded_thresh = cv::Mat::zeros(480, 640, CV_8UC1);

            // Copy the truncated array into the new cv::Mat object
            cv::Mat roi = padded_thresh(cv::Range(384, 384+thresh.rows), cv::Range::all());
            thresh.copyTo(roi);
            if (stopline) {
                cv::putText(padded_thresh, "Stopline detected!", cv::Point(static_cast<int>(w * 0.5), static_cast<int>(h * 0.5)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
            }
            if (dotted) {
                cv::putText(image, "DottedLine!", cv::Point(static_cast<int>(w*0.5), static_cast<int>(h * 0.5)), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 255), 1, cv::LINE_AA);
            }
            cv::line(image, cv::Point(static_cast<int>(center), image.rows), cv::Point(static_cast<int>(center), static_cast<int>(0.8 * image.rows)), cv::Scalar(0, 0, 255), 5);
            cv::Mat add;
            cv::cvtColor(padded_thresh, add, cv::COLOR_GRAY2BGR);
            // cv::imshow("Lane", image + add);
            // cv::waitKey(1);
        }
        if (print) {
            std::cout << "center: " << center << std::endl;
            std::cout << "thresh: " << threshold_value << std::endl;
        }
        previous_center = center;
        return center;
    }

    // NEW LANE
    std::vector<float> pixel_to_world(double x,  double y, const cv::Mat& depth_image) {
        double height = 0.16;
        double roll = 0.15;
        // Convert pixel coordinates using inverse perspective transform
        cv::Mat pixel_coord = (cv::Mat_<float>(3,1) << x, y, 1);
        cv::Mat pixel_2;
        cv::warpPerspective(pixel_coord, pixel_2, invMatrix, pixel_coord.size(), cv::INTER_LINEAR);
        // cv::Mat original_pixel_coord = transMatrix.inv() * pixel_coord;
        double depthy = pixel_2.at<float>(0);
        // Access depth value from depth image
        // Calculate world coordinates
        // double map_y = sqrt(pow(depth_value, 2) - pow(height, 2));
        // double map_x = (original_pixel[0] - CAMERA_PARAMS.at("cx")) * depth_value / CAMERA_PARAMS.at("fx") + roll * depth_value;
        // // Create vector and populate it with the world coordinates
        std::vector<float> world_coords(2);
        // world_coords[0] = static_cast<float>(map_x);
        // world_coords[1] = static_cast<float>(map_y);
        return world_coords;
    }
    
    cv::Mat viz3(const cv::Mat& binary_warped,
        const cv::Mat& non_warped, 
        const std::tuple<int, std::vector<double>, std::vector<double>, bool, int, bool>& ret, 
        const std::vector<double> waypoints, 
        const std::vector<int>& y_Values, 
        bool IPM = true) 

        {
        // Grab variables from ret tuple
        auto left_fit = std::get<1>(ret);
        auto right_fit = std::get<2>(ret);
        auto number_of_fit = std::get<0>(ret);

        // Generate y values for plotting
        std::vector<double> ploty;
        for (int i = 0; i < binary_warped.rows; ++i) {
            ploty.push_back(i);
        }

        // Create an empty image
        cv::Mat result(binary_warped.size(), CV_8UC3, cv::Scalar(0, 0, 0));

        // Update values only if they are not None
        std::vector<double> left_fitx, right_fitx;
        if (number_of_fit == 1 || number_of_fit == 2) {
            for (double y : ploty) {
                left_fitx.push_back(left_fit[0] + y * left_fit[1] + left_fit[2] * (y*y));
            }
        }
        if (number_of_fit == 3 || number_of_fit == 2) {
            for (double y : ploty) {
                right_fitx.push_back(right_fit[0] + y * right_fit[1] + right_fit[2] * (y*y));

            }
        }

        if (number_of_fit == 1 || number_of_fit == 2) {
            std::vector<cv::Point> left_points;
            for (size_t i = 0; i < left_fitx.size(); ++i) {
                left_points.push_back(cv::Point(left_fitx[i], ploty[i]));
            }
            cv::polylines(result, left_points, false, cv::Scalar(255, 255, 0), 15);
        }
        if (number_of_fit == 3 || number_of_fit == 2) {
            std::vector<cv::Point> right_points;
            for (size_t i = 0; i < right_fitx.size(); ++i) {
                right_points.push_back(cv::Point(right_fitx[i], ploty[i]));
            }
            cv::polylines(result, right_points, false, cv::Scalar(255, 255, 0), 15);
        }

        // Draw waypoints
        for (size_t i = 0; i < y_Values.size(); ++i) {
            int x = static_cast<int>(waypoints[i]);
            int y = y_Values[i];
            cv::circle(result, cv::Point(x, y), 5, cv::Scalar(0, 0, 255), -1);
        }

        // // Draw stop line
        // if (stop_line) {
        //     cv::line(result, cv::Point(0, stop_index), cv::Point(639, stop_index), cv::Scalar(0, 0, 255), 2);
        // }
        if (IPM) {
            cv::addWeighted( result, 0.95,binary_warped, 0.3, 0, result);
        }
        
        if (!IPM) {
            // Apply inverse perspective transform
            cv::Mat result_ipm;
            cv::warpPerspective(result, result_ipm, invMatrix, binary_warped.size(), cv::INTER_LINEAR);
            cv::addWeighted( non_warped, 0.3,result_ipm, 0.95, 0, result);
        }

        // if (stop_line) {
        //     cv::putText(result, "Stopline detected!", cv::Point(64, 48), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        // }

        // if (cross_walk) {
        //     cv::putText(result, "Crosswalk detected!", cv::Point(128, 96), cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(255, 255, 255), 1, cv::LINE_AA);
        // }

        return result;
    }

    void printTuple(const std::tuple<int, std::vector<double>, std::vector<double>, bool, int, bool>& ret) {
        std::cout << "Contents of the tuple ret:" << std::endl;
        std::cout << "number_of_fits: " << std::get<0>(ret) << std::endl;

        std::cout << "left_fit: ";
        for (const auto& val : std::get<1>(ret)) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        std::cout << "right_fit: ";
        for (const auto& val : std::get<2>(ret)) {
            std::cout << val << " ";
        }
        std::cout << std::endl;

        std::cout << "stop_line: " << std::boolalpha << std::get<3>(ret) << std::endl;
        std::cout << "stop_index: " << std::get<4>(ret) << std::endl;
        std::cout << "cross_walk: " << std::boolalpha << std::get<5>(ret) << std::endl;
    }


    int getIPM(const cv::Mat& inputImage, cv::Mat& outputImage) {
        if (!maps_initialized) {
            std::cerr << "Error: Maps not initialized." << std::endl;
            return 0;
        }

        static cv::Mat remapped, result;
        // Apply the precomputed maps to remap the image
        cv::remap(inputImage, remapped, map1, map2, cv::INTER_LINEAR, cv::BORDER_CONSTANT);

        // Apply the perspective warp transformation
        cv::warpPerspective(remapped, result, transMatrix, inputImage.size(), cv::INTER_LINEAR);
        
        outputImage = result;
        return 1;
    }



    void getLanes(const cv::Mat &inputImage, cv::Mat &outputImage) {
        cv::Mat imageHist;
        cv::calcHist(&inputImage, 1, 0, cv::Mat(), imageHist, 1, &inputImage.rows, 0);
        double minVal, maxVal;
        cv::minMaxLoc(imageHist, &minVal, &maxVal);
        int threshold_value = std::min(std::max(static_cast<int>(maxVal) - 75, 30), 200);
        static cv::Mat binary_thresholded = cv::Mat::zeros(inputImage.size(), CV_8UC1);
        cv::threshold(inputImage, binary_thresholded, threshold_value, 255, cv::THRESH_BINARY);
        outputImage =  binary_thresholded;
    }

    std::vector<double> getWaypoints(std::tuple<int,std::vector<double>, std::vector<double>, bool, int, bool> ret, std::vector<int> &y_Values) {
        int offset = 175;
        std::vector<double> wayPoint(y_Values.size()); // Resized wayPoint to match the size of y_Values
            std::vector<double> L_x(y_Values.size());     // Resized L_x
        std::vector<double> R_x(y_Values.size());     // Resized R_x
        int number_of_fits = std::get<0>(ret);
        std::vector<double> fit_L = std::get<1>(ret);
        std::vector<double> fit_R = std::get<2>(ret);
        if (number_of_fits == 2) {
            for (size_t i = 0; i < y_Values.size(); ++i) {
                L_x[i] = fit_L[0] + y_Values[i]*fit_L[1] + fit_L[2]*(y_Values[i])*(y_Values[i]);
                R_x[i] = fit_R[0] + y_Values[i]*fit_R[1] + fit_R[2]*(y_Values[i])*(y_Values[i]);
                wayPoint[i] = 0.5*(L_x[i] + R_x[i]);
                wayPoint[i] = static_cast<int>(std::max(0.0, std::min(static_cast<double>(wayPoint[i]), 639.0)));
            }
        } else if (number_of_fits == 1) {
            for (size_t i = 0; i < y_Values.size(); ++i) {
                L_x[i] = (offset - (480 - y_Values[i])*0.05) + fit_L[0] + y_Values[i]*fit_L[1] + fit_L[2]*(y_Values[i])*(y_Values[i]);
                wayPoint[i] = std::max(0.0, std::min(L_x[i], 639.0));
            }
        } else if (number_of_fits == 3) {
            for (size_t i = 0; i < y_Values.size(); ++i) {
                R_x[i] = - (offset - (480 - y_Values[i])*0.08) + fit_R[0] + y_Values[i]*fit_R[1] + fit_R[2]*(y_Values[i])*(y_Values[i]);
                wayPoint[i] = std::max(0.0, std::min(R_x[i], 639.0));
            }
        // } else if (wayLines["stop_line"] == "0") {
        //     for (size_t i = 0; i < y_Values.size(); ++i) {
        //         wayPoint[i] = 320;
        //     }
        } else {
            for (size_t i = 0; i < y_Values.size(); ++i) {
                wayPoint[i] = 320;
            }
        }
        return wayPoint;
    }
    void plot_polynomial(const std::vector<double> &a3, const std::vector<double> &a2, const std::vector<double> &a4, const std::vector<int> &yY) {
        // Create a range of x-values
        std::vector<int> x_values;
        std::vector<double> x_2;
        for (int x = 0; x <= 640; x += 1) {
            x_values.push_back(x);
        }

        // Evaluate the polynomial for each x-value
        std::vector<double> yR_values(x_values.size());
        std::vector<double> yL_values(x_values.size());
        for (size_t i = 0; i < x_values.size(); ++i) {
            yL_values[i] = a3[0] + x_values[i]*a3[1] + a3[2]*(x_values[i])*(x_values[i]);
            yR_values[i] = a2[0] + x_values[i]*a2[1] + a2[2]*(x_values[i])*(x_values[i]);
        }

    }


    void displayHistogram(const cv::Mat& histogram) {
        // Convert histogram data to vector for plotting
        std::vector<int> histData;
        
    // Copy the histogram data to a vector
        for (int i = 0; i < histogram.cols; ++i) {
            histData.push_back(histogram.at<int>(0, i));
        }
    }



    std::vector<int> find_center_indices(const cv::Mat & histogram, int threshold) { // Find the center indices of potential lane lines
        
        std::vector<int> hist;
        
        for (int i = 0; i < histogram.cols; ++i) {
            hist.push_back(histogram.at<int>(0, i));
        }

        std::vector<std::vector<int>> valid_groups;
        
        std::vector<int> above_threshold; // Container for histogram values that are above the threshold

        for (int i = 0; i < hist.size(); ++i) {     // Parse through to check for values above threshold
            if (hist[i] > threshold) {
                above_threshold.push_back(i);       // Append values that are above
            }
        }

        std::vector<std::vector<int>> consecutive_groups;   // Container for consecutive groups 

        for (int i = 0; i < above_threshold.size();) {      // Parse through indices of values above threshold
            int j = i;
            while (j < above_threshold.size() && above_threshold[j] - above_threshold[i] == j - i) {
                ++j;
            }
            if (j - i >= 5) {
                consecutive_groups.push_back(std::vector<int>(above_threshold.begin() + i, above_threshold.begin() + j));   // Retain consectuive groups
            }
            i = j;
        }

        // Iterate over consecutive_groups and find ones that are five continuous pixels or more
        for (const std::vector<int>& group : consecutive_groups) {
            if (group.size() >= 5) {
                valid_groups.push_back(group);
            }
        }


        // Find the center index for each valid group
        std::vector<int> center_indices;
        for (const auto& group : valid_groups) {
                int front = group.front();
                int back = group.back();
                int midpoint_index = group.front() + 0.5*(group.back() - group.front());
                center_indices.push_back(midpoint_index);
        }

        return center_indices;
    }

    int find_stop_line(const cv::Mat& image, int threshold) { // Function to identify presence of stop line
        int width = 300;
        stop_line = false;
        int stop_loc = -1;
        std::vector<int> hist;
        cv::Mat roi = image(cv::Range::all(), cv::Range(0, 639));
        cv::reduce(roi, horistogram, 1, cv::REDUCE_SUM, CV_32S);


        for (int i = 0; i < horistogram.rows; ++i) {
            hist.push_back(static_cast<int>(horistogram.at<int>(0,i)/255));
                if (hist[i] >= width) {
                // stop_line = true;
                stop_loc = i;
                // above_width_indices.push_back(i);
                // stop_line = true;
            }

        }
        return stop_loc;
    }

    bool check_cross_walk(const cv::Mat& image, int stop_index) {
        // Compute the density of non-white pixels
        cv::Mat roi = image(cv::Range(0, stop_index), cv::Range::all());
        double density = static_cast<double>(cv::countNonZero(roi)) / (roi.rows * roi.cols);

        // Check if the density exceeds the threshold (0.3)
        if (density > 0.3) {
            return true;
        } else {
            return false;
        }
    }

    std::vector<int> concatenate(const std::vector<std::vector<int>>& arrays) {
        std::vector<int> result;
        for (const auto& array : arrays) {
            result.insert(result.end(), array.begin(), array.end());
        }
        return result;
    }

    std::vector<int> find_closest_pair(const std::vector<int>& indices, int lane_width) {

        int n = indices.size();     // size of input array

        if (n < 2) {        // check to see if at least two lane lines
            throw std::invalid_argument("Array must have at least two elements");
        }

        int min_diff = std::numeric_limits<int>::max();
        std::vector<int> result_pair(2);
        result_pair[0] = indices[0];
        result_pair[1] = indices[1];

        for (int i = 0; i < n - 1; ++i) {       // iterate over different pairs
            for (int j = i + 1; j < n; ++j) {
                int current_diff = std::abs(std::abs(indices[i] - indices[j]) - lane_width);        // check for how close to optimal distance the current distance is
                if (current_diff < min_diff) {
                    min_diff = current_diff;        // compare current pair difference with optimal difference
                    result_pair[0] = indices[i];
                    result_pair[1] = indices[j];
                }
            }
        }

        return result_pair;
    }

    std::vector<double> convertToArray(const alglib::real_1d_array& arr) {  // Convert between alglib 1d array and std::vector
        std::vector<double> vec;        // Declare vector
        int size = arr.length();        
        vec.reserve(size);  
        for (int i = 0; i < size; ++i) {    // Iterate over to to transform
            vec.push_back(arr[i]);
        }
        return vec;
    }

    std::tuple<int,std::vector<double>, std::vector<double>, bool, int, bool> line_fit_2(cv::Mat binary_warped){
        auto start1 = high_resolution_clock::now();
        // Declare variables to be used
        static int lane_width = 350;        // HARD CODED LANE WIDTH
        static int n_windows = 9;           // HARD CODED WINDOW NUMBER FOR LANE PARSING
        cv::Mat histogram;
        static int threshold = 2000;       // HARD CODED THRESHOLD
        int leftx_base = 0;
        int rightx_base = 640;
        static std::tuple<int,std::vector<double>, std::vector<double>, bool, int, bool> ret;
        // Tuple variables  --- Initialize and declare
        int number_of_fits = 0;
        std::vector<double> left_fit = {0.0};
        std::vector<double> right_fit = {0.0};
        bool stop_line = false;
        int stop_index = 0;
        bool cross_walk = false;
        cv::reduce(binary_warped(cv::Range(200, 480), cv::Range::all()) / 2, histogram, 0, cv::REDUCE_SUM, CV_32S);

        auto start2 = high_resolution_clock::now();
        std::vector<int> indices = find_center_indices(histogram, threshold);               // Get the center indices

        auto start3 = high_resolution_clock::now();
        int size_indices = indices.size();      // Number of lanes detected

        if(size_indices == 0){                  // Check to see if lanes detected, if not return
            number_of_fits = 0;
            ret = std::make_tuple(number_of_fits,left_fit,right_fit,stop_line,stop_index,cross_walk);
            return ret;
        }

        if(size_indices == 1){                  // If only one lane line is detected
            if(indices[0] < 320){               // Check on which side of the car it is
                number_of_fits = 1;      // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
                leftx_base = indices[0];
                rightx_base = 0;
            }
            else {                  
                number_of_fits = 3; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
                leftx_base = 0;
                rightx_base = indices[0];
            }
        }

        else {                      
            if(size_indices > 2){   // If more than one lane line, check for closest pair of lane lines 
                std::vector<int> closest_pair = find_closest_pair(indices,lane_width);
                indices[0] = closest_pair[0];       // Initialize the start of the lane line at bottom of the screen
                indices[1] = closest_pair[1];
            }
            int delta = std::abs(indices[0]-indices[1]);        // Check to see if the two lane lines are close enough to be the same
            if(delta < 160){
                indices[0] = 0.5*(indices[0]+indices[1]);
                if(indices[0] < 320){
                    number_of_fits = 1;      // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
                    leftx_base = indices[0];
                    rightx_base = 0;
                }
                else{
                    number_of_fits = 3; // NOTE : 1-LEFT FIT, 2- BOTH FITS, 3 - RIGHT FIT
                    leftx_base = 0;
                    rightx_base = indices[0];  
                }
            }
            else{
            leftx_base = indices[0];       // Initialize the start of the lane line at bottom of the screen
            rightx_base = indices[1];
            number_of_fits = 2;                 // Set number of fits as a reference
            }

        }

        int window_height = static_cast<int>(binary_warped.rows / n_windows);        // Caclulate height of parsing windows
        // Find nonzero pixel locations
        std::vector<cv::Point> nonzero;

        cv::findNonZero(binary_warped, nonzero);    // Find nonzero values in OpenCV point format
        
        auto start4 = high_resolution_clock::now();

        // Separate x and y coordinates of nonzero pixels
        std::vector<int> nonzeroy, nonzerox;
        for (size_t i = 0; i < nonzero.size(); i += 2) { // Increment index by 2
            nonzeroy.push_back(nonzero[i].y);
            nonzerox.push_back(nonzero[i].x);

        }

        auto start5 = high_resolution_clock::now();
        // Current positions to be updated for each window
        int leftx_current = leftx_base; // Assuming leftx_base is already defined
        int rightx_current = rightx_base; // Assuming rightx_base is already defined

        // Set the width of the windows +/- margin
        static int margin = 50;

        // Set minimum number of pixels found to recenter window
        static int minpix = 50;

        // Create empty vectors to receive left and right lane pixel indices
        std::vector<int> left_lane_inds;
        std::vector<int> right_lane_inds;

        for (int window = 0; window < n_windows; ++window) {
            // Identify window boundaries in y
            int win_y_low = binary_warped.rows - (window + 1) * window_height;
            int win_y_high = binary_warped.rows - window * window_height;

            // LEFT LANE
            if (number_of_fits == 1 || number_of_fits == 2) {
                int win_xleft_low = leftx_current - margin;     // Bounding boxes around the lane lines
                int win_xleft_high = leftx_current + margin;
                int sum_left = 0;
                std::vector<int> good_left_inds;
                for (size_t i = 0; i < nonzerox.size(); ++i) {  // Parse through and only select pixels within the bounding boxes
                    if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high &&
                        nonzerox[i] >= win_xleft_low && nonzerox[i] < win_xleft_high) {
                        good_left_inds.push_back(i);            // Keep pixels within the boxes
                        sum_left += nonzerox[i];
                    }
                }

                left_lane_inds.insert(left_lane_inds.end(), good_left_inds.begin(), good_left_inds.end());      // Append all good indices together

                if (good_left_inds.size() > minpix) {       // Recenter mean for the next bounding box
                    int mean_left = sum_left/good_left_inds.size();
                    leftx_current =  mean_left;
                }
            }

            // RIGHT LANE
            if (number_of_fits == 3 || number_of_fits == 2) {
                int win_xright_low = rightx_current - margin;   // Bounding boxes around the lane lines
                int win_xright_high = rightx_current + margin;
                int sum_right = 0;
                std::vector<int> good_right_inds;
                for (size_t i = 0; i < nonzerox.size(); ++i) {  // Parse through and only select pixels within the bounding boxes
                    if (nonzeroy[i] >= win_y_low && nonzeroy[i] < win_y_high &&
                        nonzerox[i] >= win_xright_low && nonzerox[i] < win_xright_high) {
                        good_right_inds.push_back(i);           // Keep pixels within the boxes
                        sum_right += nonzerox[i];
                    }
                }

                right_lane_inds.insert(right_lane_inds.end(), good_right_inds.begin(), good_right_inds.end());  // Append all good indices together

                if (good_right_inds.size() > minpix) {          // Keep pixels within the boxes
                    int mean_right = sum_right / good_right_inds.size();
                    rightx_current = mean_right;
                }
            }

        }

        // auto start6 = high_resolution_clock::now();
        // Declare vectors to contain the pixel coordinates to fit
        std::vector<double> leftx;
        std::vector<double> lefty;
        std::vector<double> rightx;
        std::vector<double> righty;
        // Define the degree of the polynomial
        int m = 3;

        if (number_of_fits == 1 || number_of_fits == 2) {
            // Concatenate left_lane_inds if needed
            // left_lane_inds = concatenate(left_lane_inds);

            // Populate leftx and lefty vectors
            for (int idx : left_lane_inds) {
                leftx.push_back(nonzerox[idx]);
                lefty.push_back(nonzeroy[idx]);            
            }

            // Perform polynomial fitting
            static alglib::real_1d_array x_left, y_left;           // Declare alglib array type
            x_left.setcontent(leftx.size(), leftx.data());  // Populate X array
            y_left.setcontent(lefty.size(), lefty.data());  // Populate Y array
            static alglib::polynomialfitreport rep_left;
            static alglib::barycentricinterpolant p_left;
            alglib::polynomialfit(y_left, x_left, m, p_left, rep_left);     // Perform polynomial fit
            // Convert polynomial coefficients to standard form
            static alglib::real_1d_array a1;
            alglib::polynomialbar2pow(p_left, a1);
            left_fit = convertToArray(a1);      // Convert back to std::vector 
        }


        if (number_of_fits == 3 || number_of_fits == 2) {
            // Concatenate right_lane_inds if needed
            // left_lane_inds = concatenate(left_lane_inds);

            // Populate rightx and righty vectors
            for (int idx : right_lane_inds) {
                rightx.push_back(nonzerox[idx]);
                righty.push_back(nonzeroy[idx]);
            }

            // Perform polynomial fitting
            static alglib::real_1d_array x_right, y_right;             // Declare alglib array type
            x_right.setcontent(rightx.size(), rightx.data());   // Populate X array
            y_right.setcontent(righty.size(), righty.data());   // Populate Y array
            static alglib::polynomialfitreport rep_right; 
            static alglib::barycentricinterpolant p_right;
            alglib::polynomialfit(y_right, x_right, m, p_right, rep_right);     // Perform polynomial fit
            // Convert polynomial coefficients to standard form
            static alglib::real_1d_array a3;
            alglib::polynomialbar2pow(p_right, a3);
            right_fit = convertToArray(a3);     // Convert back to std::Vector 
        }
        // auto stop = high_resolution_clock::now();
        // auto duration1 = duration_cast<microseconds>(start2 - start1);
        // auto duration2 = duration_cast<microseconds>(start3 - start2);
        // auto duration3 = duration_cast<microseconds>(start4 - start3);
        // auto duration4 = duration_cast<microseconds>(start5 - start4);
        // auto duration5 = duration_cast<microseconds>(start6 - start5);
        // auto duration6 = duration_cast<microseconds>(stop - start6);
        // printf("line_fit_2: duration1: %ld\nline_fit_2: duration2: %ld\nline_fit_2: duration3: %ld\nline_fit_2: duration4: %ld\nline_fit_2: duration5: %ld\nline_fit_2: duration6: %ld\n", duration1.count(), duration2.count(), duration3.count(), duration4.count(), duration5.count(), duration6.count());

        // Make and return tuple of required values
        ret = std::make_tuple(number_of_fits,left_fit,right_fit,stop_line,stop_index,cross_walk);
        return ret;
    }

};