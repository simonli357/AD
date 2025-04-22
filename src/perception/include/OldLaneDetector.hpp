#pragma once

#include <ros/ros.h>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/Image.h>
#include <chrono>
#include "utils/Lane2.h"
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
#include <numeric>

using namespace std::chrono;

class OldLaneDetector {
public:
    OldLaneDetector(bool showflag, bool printflag) : previous_center(320), showflag(showflag), printflag(printflag)
    {
        image = cv::Mat::zeros(480, 640, CV_8UC1);
        stopline = false;
        dotted = false;
    }

    double num_iterations = 1;
    double previous_center;
    double total;
    cv::Mat maskh, masks, image, maskd;
    bool stopline, dotted, publish;
    int stopline_dist = -1;
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
    bool showflag, printflag;
    
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

    double optimized_histogram(const cv::Mat& image, bool show = false, bool print = false, bool robust=true) {
        stopline = false;

        // cv::cvtColor(image, img_gray, cv::COLOR_BGR2GRAY);
        // // apply maskh
        // img_roi = img_gray(cv::Rect(0, 384, 640, 96));
        // cv::minMaxLoc(img_roi, &minVal, &maxVal, &minLoc, &maxLoc);
        double threshold_value = std::min(std::max(maxVal - 55.0, 30.0), 200.0);
        // cv::threshold(img_roi, thresh, threshold_value, 255, cv::THRESH_BINARY);

        img_roi = image(cv::Rect(0, 384, 640, 48));

        // DISPLAY
        // cv::imshow("img_roi", img_roi);
        // cv::waitKey(1);

        hist = cv::Mat::zeros(1, w, CV_32SC1);
        // cv::reduce(thresh, hist, 0, cv::REDUCE_SUM, CV_32S);
        cv::reduce(img_roi, hist, 0, cv::REDUCE_SUM, CV_32S);

        extract_lanes(hist);
        std::vector<double> centers;
        // std::cout << "lane size: " << lanes.size() << std::endl;
        // for(auto lane: lanes) {
        //     std::cout << lane << " ";
        // }
        // std::cout << std::endl;
        for (size_t i = 0; i < lanes.size() / 2; ++i) {
            if (abs(lanes[2 * i] - lanes[2 * i + 1])>350 && threshold_value>50){
                stopline = true;
                // Calculate the distance to the stopline
                static cv::Mat row_sums;
                // cv::reduce(thresh, row_sums, 1, cv::REDUCE_SUM, CV_32S);
                cv::reduce(img_roi, row_sums, 1, cv::REDUCE_SUM, CV_32S);
                for (int r = 0; r < row_sums.rows; ++r) {
                    if (row_sums.at<int>(r, 0) > 0) { // Stopline detected in this row
                        stopline_dist = row_sums.rows - r; // Distance from bottom of ROI
                        break;
                    }
                }
                if (!show) return w / 2.0;
            } else {
                stopline = false;
                stopline_dist = -1;
            }
            if (3 < abs(lanes[2 * i] - lanes[2 * i + 1])) {
                centers.push_back((lanes[2 * i] + lanes[2 * i + 1]) / 2.0);
            }
        }

        // std::cout << "centers size: " << centers.size() << std::endl;
        // for(auto center: centers) {
        //     std::cout << center << " ";
        // }
        // std::cout << std::endl;
        double center = -1;
        if (centers.size() == 2) center = (centers[0] + centers.back()) / 2;

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
            cv::line(image, cv::Point(static_cast<int>(center), image.rows), cv::Point(static_cast<int>(center), static_cast<int>(0.8 * image.rows)), cv::Scalar(255, 255, 255), 5);
            cv::Mat add;
            cv::cvtColor(padded_thresh, add, cv::COLOR_GRAY2BGR);
            cv::Mat image_bgr = cv::Mat::zeros(480, 640, CV_8UC3);
            cv::cvtColor(image, image_bgr, cv::COLOR_GRAY2BGR);
            cv::imshow("Lane", image_bgr + add);
            cv::waitKey(1);
        }
        if (print) {
            std::cout << "center: " << center << std::endl;
            std::cout << "thresh: " << threshold_value << std::endl;
        }
        previous_center = center;
        return center;
    }

};