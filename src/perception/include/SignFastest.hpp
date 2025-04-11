#pragma once

#include "TcpClient.hpp"
#include "ros/ros.h"
#include "yolo-fastestv2.h"
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include <chrono>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include <utils/Sign.h>
#include <mutex>
#include "engine.h"
#include "yolov8.h"
#include <memory>
#include <eigen3/Eigen/Dense>
#include "utils/constants.h"
#include "utils/helper.h"
#include "LightClassifier.hpp"

using namespace std::chrono;
using namespace VehicleConstants;

class SignFastest {
    public:
        SignFastest(ros::NodeHandle& nh, bool real = false) : 
            real(real)
        {
            std::cout.precision(4);

            if (!nh.getParam("/use_emergency", use_emergency)) {
                ROS_WARN("Failed to get 'use_emergency' parameter. Defaulting to false.");
                use_emergency = false;
            }
            bool use_tcp = false;
            if (!nh.getParam("/use_tcp", use_tcp)) {
                ROS_WARN("Failed to get 'use_tcp' parameter. Defaulting to false.");
                use_tcp = false;
            }
            if(use_tcp) {
                ROS_INFO("Attempting to create TCP client...");
                std::string ip_address;
                if (!nh.getParam("/ip", ip_address)) {
                    ROS_ERROR("Failed to get 'ip_address' parameter. TCP client not created.");
                    tcp_client = nullptr;
                } else {
                    tcp_client = std::make_unique<TcpClient>(false, "sign_node_client", ip_address);
                    ROS_INFO("TCP client created successfully.");
                }
            } else {
                tcp_client = nullptr;
                ROS_INFO("TCP client not created.");
            }

            nh.param("class_names", class_names, std::vector<std::string>());
            if(!nh.param("confidence_thresholds", confidence_thresholds, std::vector<float>(13))) {
                ROS_WARN("Failed to get 'confidence_thresholds' parameter.");
            } else {
                std::cout << "loaded confidence_thresholds size: " << confidence_thresholds.size() << std::endl;
            }
            if (!nh.param("max_distance_thresholds", distance_thresholds, std::vector<float>(13))) {
                ROS_WARN("Failed to get 'max_distance_thresholds' parameter.");
            } else {
                std::cout << "loaded distance_thresholds size: " << distance_thresholds.size() << std::endl;
            }
            if (!nh.param("counter_thresholds", counter_thresholds, std::vector<int>(13))) {
                ROS_WARN("Failed to get 'counter_thresholds' parameter.");
            } else {
                std::cout << "loaded counter_thresholds size: " << counter_thresholds.size() << std::endl;
            }
            for(int i = 0; i < class_names.size(); i++) {
                ROS_INFO("class_names: %s, confidence thresh: %.3f, distance_thresholds: %.3f, counter_thresholds: %d", class_names[i].c_str(), confidence_thresholds[i], distance_thresholds[i], counter_thresholds[i]);
            }
            std::string nodeName = ros::this_node::getName();
            nh.param(nodeName+"/showFlag", show, false);
            nh.param(nodeName+"/printFlag", print, false);
            nh.param(nodeName+"/printDuration", printDuration, false); //printDuration
            nh.param(nodeName+"/hasDepthImage", hasDepthImage, false);
            nh.param(nodeName+"/real", real, false);
            nh.param(nodeName+"/pub", publish, false);
            nh.param(nodeName+"/ncnn", ncnn, false);
            nh.param(nodeName+"/min_ground_distance", min_ground_distance, 429.0);

            std::string model;
            nh.param("ncnn_model", model, std::string("sissi753-opt"));
            std::cout << "showFlag: " << show << std::endl;
            std::cout << "printFlag: " << print << std::endl;
            std::cout << "printDuration: " << printDuration << std::endl;
            std::cout << "class_names: " << class_names.size() << std::endl;
            std::cout << "confidence_thresholds: " << confidence_thresholds.size() << std::endl;
            std::cout << "distance_thresholds: " << distance_thresholds.size() << std::endl;
            std::cout << "model: " << model << std::endl;
            printf("hasDepthImage: %s\n", hasDepthImage ? "true" : "false");
            
            sign_counter.resize(OBJECT_COUNT, 0);

            if (ncnn) {
                yolo_fastestv2_api = std::make_unique<yoloFastestv2>();
                std::string filePathBin = helper::getSourceDirectory() + "/../../../perception/models/ncnn/" + model + ".bin";
                std::string filePathParam = helper::getSourceDirectory() + "/../../../perception/models/ncnn/" + model + ".param";
                const char* bin = filePathBin.c_str();
                const char* param = filePathParam.c_str();

                yolo_fastestv2_api->loadModel(param, bin);
            } else {
                std::string model_name;
                nh.getParam("/v8_model", model_name); 
                // model_name = "v2originalTRT"; 
                std::string current_path = helper::getSourceDirectory();
                std::string modelPath = current_path + "/../../../perception/models/trt/" + model_name + ".onnx";
                yolov8 = std::make_unique<YoloV8>(modelPath, config);
            }

            pub = nh.advertise<utils::Sign>("sign", 10);
            // std::cout <<"pub created" << std::endl;

            processed_image_pub = nh.advertise<sensor_msgs::Image>("processed_image", 10);
        }
        
        // std_msgs::Float32MultiArray sign_msg;
        utils::Sign sign_msg;

        static constexpr int OBJECT_COUNT = 13;
        double min_ground_distance = 429; // in mm
        // private:
        std::unique_ptr<yoloFastestv2> yolo_fastestv2_api;
        LightClassifier light_classifier;
    
        std::unique_ptr<TcpClient> tcp_client;

        ros::Publisher pub;
        ros::Publisher processed_image_pub;
        sensor_msgs::ImagePtr processed_image_msg;
        bool show;
        bool print;
        bool printDuration;
        bool hasDepthImage;
        bool real;
        bool publish;
        bool ncnn;
        bool use_emergency = false;

        cv::Mat normalizedDepthImage;
        cv::Mat croppedDepth;

        std::vector<TargetBox> boxes;
        std::vector<TargetBox> boxes_depth;
        high_resolution_clock::time_point start;
        high_resolution_clock::time_point stop;
        std::chrono::microseconds duration;

        std::vector<double> depths;
        std::vector<float> confidence_thresholds;
        std::vector<float> distance_thresholds;
        std::vector<int> counter_thresholds;
        std::vector<std::string> class_names;
        std::vector<int> sign_counter;

        std::mutex mutex;

        std::vector<cv::Rect> tmp_boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        TargetBox tmp_box;

        //yolov8 engine inference related
        std::vector<Object> detected_objects;
        // YoloV8Config config;
        YoloV8Config config;
        std::unique_ptr<YoloV8> yolov8;

        static constexpr double SIGN_H2D_RATIO = 31.57;
        static constexpr double LIGHT_W2D_RATIO = 41.87;
        static constexpr double CAR_H2D_RATIO = 90.15;

        bool distance_makes_sense(double distance, int class_id, float x1, float y1, float x2, float y2) {
            if (distance > distance_thresholds[class_id]) return false;
            double expected_dist;
            double width = x2 - x1;
            double height = y2 - y1;
            if (class_id == OBJECT::CAR) {
                expected_dist =  CAR_H2D_RATIO / height;
            } else if (class_id == OBJECT::LIGHTS || class_id == OBJECT::GREENLIGHT || class_id == OBJECT::YELLOWLIGHT || class_id == OBJECT::REDLIGHT) {
                expected_dist = LIGHT_W2D_RATIO / width;
            } else { // sign
                expected_dist = SIGN_H2D_RATIO / height;
            }
            if (distance > expected_dist * 3 || distance < expected_dist * 1/3) return false;
            return true;
        }
        bool detect_emergency_obstacle(const cv::Mat& depthImage) {
            if (!use_emergency) return false;
            if (depthImage.empty() || depthImage.type() != CV_32FC1) {
                std::cerr << "Invalid depth image!" << std::endl;

                return false; // Return false for invalid input
            }
            // if (depthImage.type() != CV_32FC1) {
            //     std::cerr << "not cv_32fc1!" << std::endl;
            //     std::cout << "type is " << depthImage.type() << std::endl;
            //     return false; // Return false for invalid input
            // }

            int roiWidth = static_cast<int>(depthImage.cols * 0.4); // 40% of the width
            int roiHeight = static_cast<int>(depthImage.rows * 0.5); // 50% of the height
            int roiX = (depthImage.cols - roiWidth) / 2; // Center horizontally
            int roiY = static_cast<int>(depthImage.rows * 0.6) - (roiHeight / 2);
            cv::Rect roi(roiX, roiY, roiWidth, roiHeight);
            cv::Mat depthROI = depthImage(roi);

            // Find min and max values in the ROI
            double minVal, maxVal;
            cv::Point minLoc, maxLoc;
            cv::minMaxLoc(depthROI, &minVal, &maxVal, &minLoc, &maxLoc);
            cv::Point minLocGlobal(minLoc.x + roiX, minLoc.y + roiY);

            // Define threshold: 80% of the minimum value
            float thresholdValue = static_cast<float>(std::max(minVal, 30.0)) * 1.2f;

            // Count pixels below the threshold
            cv::Mat belowThresholdMask;
            cv::threshold(depthROI, belowThresholdMask, thresholdValue, 255, cv::THRESH_BINARY_INV);
            belowThresholdMask.convertTo(belowThresholdMask, CV_8U); // Convert to 8-bit for counting
            int belowThresholdCount = cv::countNonZero(belowThresholdMask);

            // cv::Mat visualization;
            // cv::normalize(depthImage, visualization, 0, 255, cv::NORM_MINMAX, CV_8U); // Normalize depth for display
            // cv::cvtColor(visualization, visualization, cv::COLOR_GRAY2BGR); // Convert to BGR for colored overlays
            // cv::rectangle(visualization, roi, cv::Scalar(0, 255, 0), 2); // Green rectangle for ROI
            // cv::circle(visualization, minLocGlobal, 5, cv::Scalar(0, 0, 255), -1); // Red filled circle
            // std::ostringstream overlayText;
            // overlayText << "Min: " << minVal << " mm, Max: " << maxVal << " mm, Below 120% Min: " << belowThresholdCount;
            // cv::putText(visualization, overlayText.str(), cv::Point(10, 30), cv::FONT_HERSHEY_SIMPLEX, 0.8, cv::Scalar(0, 255, 255), 2);
            // std::cout << "min: " << minVal << ", max: " << maxVal << ", below 80% min: " << belowThresholdCount << std::endl;
            // cv::imshow("Depth Visualization", visualization);
            // cv::waitKey(1); // Use a small delay to update the display

            if (minVal < min_ground_distance - 37 && belowThresholdCount > 0.1 * roiWidth * roiHeight) {
                return true;
            }

            return false;
        }
        
        struct DetectedBox {
            int class_id;
            float confidence;
            int x1, y1, x2, y2;
        };
        void publish_sign(const cv::Mat& image, const cv::Mat& depthImage) {
            if(printDuration) start = high_resolution_clock::now();
            sign_msg.header.stamp = ros::Time::now();
            sign_msg.data.clear();
            if (hasDepthImage && depthImage.empty()) {
                ROS_ERROR("Depth image is empty");
                return;
            }

            if(image.empty()) {
                ROS_WARN("empty image received in sign detector");
                return;
            }

            // bool emergency = detect_emergency_obstacle(depthImage);
            bool emergency = false;
            if (emergency) {
                for (int i = 0; i < NUM_VALUES_PER_OBJECT; i++) {
                    sign_msg.data.push_back(-1.0);
                }
                if (publish) {
                    pub.publish(sign_msg);
                    if (tcp_client != nullptr) tcp_client->send_sign(sign_msg.data);
                }
                if (print) ROS_INFO("Emergency obstacle detected");
                return;
            }
            
            static std::vector<int> detected_indices(OBJECT_COUNT, 0);
            std::fill(detected_indices.begin(), detected_indices.end(), 0);
            
            int hsy = 0; // count of detections that yield a valid refined depth
            std::vector<DetectedBox> detectedBoxes;
            if (ncnn) {
                yolo_fastestv2_api->detection(image, boxes);
                for (const auto &box : boxes) {
                    int class_id = box.cate;
                    detected_indices[class_id] = 1;
                    sign_counter[class_id]++;
                    if (sign_counter[class_id] < counter_thresholds[class_id])
                        continue;
                    if (box.score < confidence_thresholds[class_id])
                        continue;
                    DetectedBox db;
                    db.class_id = class_id;
                    db.confidence = box.score;
                    db.x1 = box.x1;
                    db.y1 = box.y1;
                    db.x2 = box.x2;
                    db.y2 = box.y2;
                    detectedBoxes.push_back(db);
                }
            } else {
                detected_objects = yolov8->detectObjects(image);
                for (struct Object& box : detected_objects) {
                    int class_id = box.label;
                    detected_indices[class_id] = 1;
                    sign_counter[class_id]++;
                    if (sign_counter[class_id] < counter_thresholds[class_id]) {
                        // ROS_INFO("%s detected but counter is only %d", class_names[class_id].c_str(), sign_counter[class_id]);
                        continue;
                    }
                    float confidence = box.probability;
                    int x1 = box.rect.x;
                    int y1 = box.rect.y;
                    int x2 = box.rect.x + box.rect.width;
                    int y2 = box.rect.y + box.rect.height;
                    if (class_id == OBJECT::LIGHTS) {
                        // Clamp the ROI coordinates to ensure they are within image bounds
                        int x1_valid = std::max(x1, 0);
                        int y1_valid = std::max(y1, 0);
                        int x2_valid = std::min(x2, image.cols);
                        int y2_valid = std::min(y2, image.rows);
                        int width = x2_valid - x1_valid;
                        int height = y2_valid - y1_valid;
                        if (width <= 0 || height <= 0) {
                            ROS_WARN("Invalid ROI for light detection, skipping classification");
                        } else {
                            cv::Rect valid_roi(x1_valid, y1_valid, width, height);
                            cv::Mat detected_light = image(valid_roi);
                            auto light_color = light_classifier.classify(detected_light);
                            if (light_color != LightColor::UNDETERMINED) {
                                class_id = (light_color == LightColor::RED) ? OBJECT::REDLIGHT :
                                        (light_color == LightColor::GREEN) ? OBJECT::GREENLIGHT : OBJECT::YELLOWLIGHT;
                                box.label = static_cast<int>(class_id);
                            }
                        }
                    }
                    if (box.probability < confidence_thresholds[class_id])
                        continue;
                    DetectedBox db;
                    db.class_id = class_id;
                    db.confidence = box.probability;
                    db.x1 = box.rect.x;
                    db.y1 = box.rect.y;
                    db.x2 = box.rect.x + box.rect.width;
                    db.y2 = box.rect.y + box.rect.height;
                    detectedBoxes.push_back(db);
                }
            }

            std::vector<std::pair<double, int>> depthIndices;
            for (size_t i = 0; i < detectedBoxes.size(); i++) {
                double roughDepth = computeMedianDepth(depthImage, detectedBoxes[i].x1,
                                                         detectedBoxes[i].y1, detectedBoxes[i].x2,
                                                         detectedBoxes[i].y2);
                if (roughDepth < 0)
                    roughDepth = std::numeric_limits<double>::max();
                depthIndices.push_back(std::make_pair(roughDepth, static_cast<int>(i)));
            }
            // Sort detections by rough depth (closest first)
            std::sort(depthIndices.begin(), depthIndices.end(),
                        [](const std::pair<double, int>& a, const std::pair<double, int>& b) {
                        return a.first < b.first;
                    });
            cv::Mat occlusionMask = cv::Mat::zeros(depthImage.size(), CV_8U);

            for (const auto &p : depthIndices) {
                int idx = p.second;
                DetectedBox &db = detectedBoxes[idx];
                // Compute refined (final) depth excluding overlapping regions.
                double finalDepth = computeMedianDepthExcludingOverlap(depthImage,
                                                                       db.x1, db.y1, db.x2, db.y2,
                                                                       occlusionMask) / 1000.0;
                // cv::Mat result = image.clone();
                // result.setTo(cv::Scalar(0, 0, 255), occlusionMask == 255);
                // cv::imshow("Masked Image", result);
                // cv::waitKey(0);
                if (finalDepth < 0)
                    continue;  // no valid depth remains after exclusion
                
                double expected_dist = distance_makes_sense(finalDepth, db.class_id, db.x1, db.y1, db.x2, db.y2);
                if (!expected_dist) {
                    // ROS_WARN("Distance does not make sense, expected: %.3f, got: %.3f", expected_dist, distance);
                    continue;
                }
                // Populate the sign message.
                sign_msg.data.push_back(db.x1);
                sign_msg.data.push_back(db.y1);
                sign_msg.data.push_back(db.x2);
                sign_msg.data.push_back(db.y2);
                sign_msg.data.push_back(finalDepth);
                sign_msg.data.push_back(db.confidence);
                sign_msg.data.push_back(static_cast<float>(db.class_id));
                hsy++;
            }
            for (int i = 0; i < OBJECT_COUNT; i++) {
                if (!detected_indices[i]) {
                    sign_counter[i] = 0;
                }
            }

            // if(hsy) {
            //     std_msgs::MultiArrayDimension dim;
            //     dim.label = "detections";
            //     dim.size = hsy;
            //     dim.stride = boxes.size() * NUM_VALUES_PER_OBJECT;
            //     sign_msg.layout.dim.push_back(dim); 
            // }
            // Publish Sign message
            if (publish) {
                if (sign_msg.data.size() % NUM_VALUES_PER_OBJECT != 0) {
                    ROS_WARN("Sign message size is not a multiple of %d", NUM_VALUES_PER_OBJECT);
                }
                pub.publish(sign_msg);
                if (tcp_client != nullptr) tcp_client->send_sign(sign_msg.data);
            }

            if(printDuration) {
                stop = high_resolution_clock::now();
                duration = duration_cast<microseconds>(stop - start);
                ROS_INFO("sign durations: %ld", duration.count());
            }
            if (show) {
                if (ncnn) {
                    // Normalize depth img
                    double maxVal;
                    double minVal;
                    if (hasDepthImage) {
                        cv::minMaxIdx(depthImage, &minVal, &maxVal);
                        depthImage.convertTo(normalizedDepthImage, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
                    }
                    for (int i = 0; i < boxes.size(); i++) {
                        char text[256];
                        int id = boxes[i].cate;
                        if(boxes[i].score < confidence_thresholds[id]) continue;
                        sprintf(text, "%s %.1f%%", class_names[id].c_str(), boxes[i].score * 100);
                        char text2[256];
                        if (hasDepthImage) {
                            double distance = computeMedianDepth(depthImage, boxes[i].x1, boxes[i].y1, boxes[i].x2, boxes[i].y2)/1000;
                            sprintf(text2, "%s %.1fm", class_names[id].c_str(), distance);
                        }
                        int baseLine = 0;
                        cv::Size label_size = cv::getTextSize(text, cv::FONT_HERSHEY_SIMPLEX, 0.5, 1, &baseLine);

                        int x = boxes[i].x1;
                        int y = boxes[i].y1 - label_size.height - baseLine;
                        if (y < 0)
                            y = 0;
                        if (x + label_size.width > image.cols)
                            x = image.cols - label_size.width;
                        
                        cv::rectangle(image, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                                    cv::Scalar(255, 255, 255), -1);
                        cv::putText(image, text, cv::Point(x, y + label_size.height),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                        cv::rectangle (image, cv::Point(boxes[i].x1, boxes[i].y1), 
                                    cv::Point(boxes[i].x2, boxes[i].y2), cv::Scalar(255, 255, 0), 2, 2, 0);
                        if(hasDepthImage) {
                            cv::rectangle(normalizedDepthImage, cv::Rect(cv::Point(x, y), cv::Size(label_size.width, label_size.height + baseLine)),
                                    cv::Scalar(255, 255, 255), -1);
                            cv::putText(normalizedDepthImage, text2, cv::Point(x, y + label_size.height),
                                    cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0, 0, 0));
                            cv::rectangle (normalizedDepthImage, cv::Point(boxes[i].x1, boxes[i].y1), 
                                    cv::Point(boxes[i].x2, boxes[i].y2), cv::Scalar(255, 255, 0), 2, 2, 0);
                        }
                    }
                    if(hasDepthImage) {
                        cv::imshow("normalized depth image", normalizedDepthImage);
                    }
                    cv::imshow("image", image);
                    cv::waitKey(1);
                } else {
                    cv::Mat image_copy = image.clone();
                    std::vector<double> distances;
                    for(auto &box : detected_objects) {
                        if(depthImage.empty()) {
                            ROS_WARN("displaying image. Depth image is empty");
                            distances.push_back(-1);
                            continue;
                        }
                        double distance = computeMedianDepth(depthImage, box.rect.x, box.rect.y, box.rect.x + box.rect.width, box.rect.y + box.rect.height)/1000;
                        distances.push_back(distance);
                    }
                    
                    yolov8->drawObjectLabels(image_copy, detected_objects, distances);
                    
                    processed_image_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", image_copy).toImageMsg();
                    processed_image_pub.publish(processed_image_msg);

                    cv::imshow("image", image_copy);
                    cv::waitKey(1);
                }
            }
            if (print) {
                if (ncnn) {
                    for (int i = 0; i < boxes.size(); i++) {
                        double distance = computeMedianDepth(depthImage, boxes[i].x1, boxes[i].y1, boxes[i].x2, boxes[i].y2)/1000;
                        std::cout<< "x1:" << boxes[i].x1<<", y1:"<<boxes[i].y1<<", x2:"<<boxes[i].x2<<", y2:"<<boxes[i].y2
                        <<", conf:"<<boxes[i].score<<", id:"<<boxes[i].cate<<", "<<class_names[boxes[i].cate]<<", dist:"<< 
                        distance <<", w:"<<boxes[i].x2-boxes[i].x1<<", h:"<<boxes[i].y2-boxes[i].y1 << std::endl;
                    }
                } else {
                    for (const struct Object& box : detected_objects) {
                        double distance = computeMedianDepth(depthImage, box.rect.x, box.rect.y, box.rect.x + box.rect.width, box.rect.y + box.rect.height)/1000;
                        std::cout<< "x1:" << box.rect.x<<", y1:"<<box.rect.y<<", x2:"<<box.rect.x + box.rect.width<<", y2:"<<box.rect.y + box.rect.height
                        <<", conf:"<<box.probability<<", id:"<<box.label<<", "<<class_names[box.label]<<", dist:"<< 
                        distance <<", w:"<<box.rect.width<<", h:"<<box.rect.height<< std::endl;
                    }
                
                }
            }
        }

        double computeMedianDepth(const cv::Mat& depthImage, int bbox_x1, int bbox_y1, int bbox_x2, int bbox_y2) {
            // auto start = high_resolution_clock::now();
            int x1 = std::max(0, bbox_x1);
            int y1 = std::max(0, bbox_y1);
            int x2 = std::min(depthImage.cols, bbox_x2);
            int y2 = std::min(depthImage.rows, bbox_y2);
            croppedDepth = depthImage(cv::Rect(x1, y1, x2 - x1, y2 - y1));

            if (croppedDepth.empty()) return -1;
            std::vector<double> depths;

            if (!croppedDepth.isContinuous()) {
                croppedDepth = croppedDepth.clone();
            }
            // Convert the cropped depth matrix to a single row vector of type double
            croppedDepth.reshape(1, 1).copyTo(depths);
            depths.erase(std::remove_if(depths.begin(), depths.end(), [](double depth) { return depth <= 100; }), depths.end());

            if (depths.empty()) {
                return -1; 
            }
            // printf("depths size: %ld, x1: %d, y1: %d, x2: %d, y2: %d, rows: %d, cols: %d\n", depths.size(), x1, y1, x2, y2, croppedDepth.rows, croppedDepth.cols);

            size_t index = depths.size() * 0.5;
            std::nth_element(depths.begin(), depths.begin() + index, depths.end());

            if (index % 2) { // if odd
                return depths[index / 2];
            }
            return 0.5 * (depths[(index - 1) / 2] + depths[index / 2]);
        }

		double computeMedianDepthExcludingOverlap(const cv::Mat &depthImage, int bbox_x1, int bbox_y1, 
                                                int bbox_x2, int bbox_y2, cv::Mat &occlusionMask) {
			// Clamp bounding box coordinates to the depth image dimensions.
			int x1 = std::max(0, bbox_x1);
			int y1 = std::max(0, bbox_y1);
			int x2 = std::min(depthImage.cols, bbox_x2);
			int y2 = std::min(depthImage.rows, bbox_y2);
			cv::Rect roiRect(x1, y1, x2 - x1, y2 - y1);
			if (roiRect.width <= 0 || roiRect.height <= 0)
				return -1;
			// Extract ROI from depth image and occlusion mask.
			cv::Mat depthROI = depthImage(roiRect);
			cv::Mat maskROI = occlusionMask(roiRect);

			// Collect valid depth values (exclude pixels already used and below a threshold).
			std::vector<double> validDepths;
			for (int r = 0; r < depthROI.rows; r++) {
				for (int c = 0; c < depthROI.cols; c++) {
					// If this pixel is not yet used
					if (maskROI.at<uchar>(r, c) == 0) {
						double depthValue = 0;
						// Handle common depth image types
						if (depthROI.type() == CV_16U) {
							depthValue = static_cast<double>(depthROI.at<unsigned short>(r, c));
						} else if (depthROI.type() == CV_32F) {
							depthValue = static_cast<double>(depthROI.at<float>(r, c));
						} else { // assume CV_64F
							depthValue = depthROI.at<double>(r, c);
						}
						if (depthValue > 100) { // ignore values below threshold (e.g., noise)
							validDepths.push_back(depthValue);
						}
					}
				}
			}
			if (validDepths.empty()) {
				return -1;
			}
			// Compute median
			std::sort(validDepths.begin(), validDepths.end());
			double median = 0;
			size_t n = validDepths.size();
			if (n % 2 == 1) {
				median = validDepths[n / 2];
			} else {
				median = 0.5 * (validDepths[n / 2 - 1] + validDepths[n / 2]);
			}
			// Mark the entire ROI in the occlusion mask as used.
			maskROI.setTo(255);
			return median;
		}
};
