#pragma once

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
#include <mutex>
#include "engine.h"
#include "yolov8.h"
#include <memory>

using namespace std::chrono;

std::string getSourceDirectory() {
    std::string file_path(__FILE__);  // __FILE__ is the full path of the source file
    size_t last_dir_sep = file_path.rfind('/');  // For Unix/Linux path
    if (last_dir_sep == std::string::npos) {
        last_dir_sep = file_path.rfind('\\');  // For Windows path
    }
    if (last_dir_sep != std::string::npos) {
        return file_path.substr(0, last_dir_sep);  // Extract directory path
    }
    return "";  // Return empty string if path not found
}

class SignFastest {
    public:
        SignFastest(ros::NodeHandle& nh)
        {
            std::cout.precision(4);
            nh.param("class_names", class_names, std::vector<std::string>());
            if(!nh.param("confidence_thresholds", confidence_thresholds, std::vector<float>(13))) {
                ROS_WARN("Failed to get 'confidence_thresholds' parameter.");
            } else {
                std::cout << "confidence_thresholds: " << confidence_thresholds.size() << std::endl;
            }
            if (!nh.param("max_distance_thresholds", distance_thresholds, std::vector<float>(13))) {
                ROS_WARN("Failed to get 'max_distance_thresholds' parameter.");
            } else {
                for (float val : distance_thresholds) {
                    ROS_INFO("Loaded threshold: %f", val);
                }
                for(int i = 0; i < class_names.size(); i++) {
                    ROS_INFO("class_names: %s, distance_thresholds: %f", class_names[i].c_str(), distance_thresholds[i]);
                }
            }
            if (!nh.param("counter_thresholds", counter_thresholds, std::vector<int>(13))) {
                ROS_WARN("Failed to get 'counter_thresholds' parameter.");
            } else {
                for (int val : counter_thresholds) {
                    ROS_INFO("Loaded counter threshold: %d", val);
                }
            }
            std::string nodeName = ros::this_node::getName();
            nh.param(nodeName+"/showFlag", show, false);
            nh.param(nodeName+"/printFlag", print, false);
            nh.param(nodeName+"/printDuration", printDuration, false); //printDuration
            nh.param(nodeName+"/hasDepthImage", hasDepthImage, false);
            nh.param(nodeName+"/real", real, false);
            nh.param(nodeName+"/pub", publish, false);
            nh.param(nodeName+"/ncnn", ncnn, false);

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
                std::string filePathBin = getSourceDirectory() + "/../models/ncnn/" + model + ".bin";
                std::string filePathParam = getSourceDirectory() + "/../models/ncnn/" + model + ".param";
                const char* bin = filePathBin.c_str();
                const char* param = filePathParam.c_str();

                api.loadModel(param, bin);
            } else {
                std::string model_name;
                nh.param("model_name", model_name, std::string("citycocov2lgtclab_20")); 
                // model_name = "v2originalTRT"; 
                std::string current_path = getSourceDirectory();
                std::string modelPath = current_path + "/../models/trt/" + model_name + ".onnx";
                yolov8 = std::make_unique<YoloV8>(modelPath, config);
            }

            pub = nh.advertise<std_msgs::Float32MultiArray>("sign", 10);
            std::cout <<"pub created" << std::endl;
        }
        enum OBJECT {
            ONEWAY,
            HIGHWAYENTRANCE,
            STOPSIGN,
            ROUNDABOUT,
            PARK,
            CROSSWALK,
            NOENTRY,
            HIGHWAYEXIT,
            PRIORITY,
            LIGHTS,
            BLOCK,
            PEDESTRIAN,
            CAR,
        };
        static constexpr int OBJECT_COUNT = 13;
        // private:
        yoloFastestv2 api;

        ros::Publisher pub;
        bool show;
        bool print;
        bool printDuration;
        bool hasDepthImage;
        bool real;
        bool publish;
        bool ncnn;

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
            } else if (class_id == OBJECT::LIGHTS) {
                expected_dist = LIGHT_W2D_RATIO / width;
            } else { // sign
                expected_dist = SIGN_H2D_RATIO / height;
            }
            if (distance > expected_dist * 3 || distance < expected_dist * 1/3) return false;
            return true;
        }
        int populate_sign_msg(std_msgs::Float32MultiArray& sign_msg, const cv::Mat& image, const cv::Mat& depthImage, int class_id, float confidence, int x1, int y1, int x2, int y2) {
            if (confidence >= confidence_thresholds[class_id]) {
                double distance;
                if(hasDepthImage) {
                    if (depthImage.empty()) {
                        ROS_ERROR("Depth image is empty");
                        return 0;
                    } else {
                        distance = computeMedianDepth(depthImage, x1, y1, x2, y2)/1000; // in meters
                    }
                } else {
                    distance = -1;
                }
                double expected_dist = distance_makes_sense(distance, class_id, x1, y1, x2, y2);
                if (!expected_dist) {
                    // ROS_WARN("Distance does not make sense, expected: %.3f, got: %.3f", expected_dist, distance);
                    return 0;
                }
                sign_msg.data.push_back(x1);
                sign_msg.data.push_back(y1);
                sign_msg.data.push_back(x2);
                sign_msg.data.push_back(y2);
                sign_msg.data.push_back(distance);
                sign_msg.data.push_back(confidence);
                sign_msg.data.push_back(static_cast<float>(class_id));
                return 1;
            }
            return 0;
        }
        void publish_sign(const cv::Mat& image, const cv::Mat& depthImage) {
            
            if(image.empty()) {
                ROS_WARN("empty image received in sign detector");
                return;
            }
            if(printDuration) start = high_resolution_clock::now();
            std_msgs::Float32MultiArray sign_msg;
            sign_msg.layout.data_offset = 0;

            int hsy = 0;
            
            static std::vector<int> detected_indices(OBJECT_COUNT, 0);
            std::fill(detected_indices.begin(), detected_indices.end(), 0);

            if (ncnn) {
                api.detection(image, boxes);
                for (const auto &box : boxes) {
                    int class_id = box.cate;
                    detected_indices[class_id] = 1;
                    sign_counter[class_id]++;
                    if (sign_counter[class_id] < counter_thresholds[class_id]) continue;
                    float confidence = box.score;
                    int x1 = box.x1;
                    int y1 = box.y1;
                    int x2 = box.x2;
                    int y2 = box.y2;
                    if (populate_sign_msg(sign_msg, image, depthImage, class_id, confidence, x1, y1, x2, y2)) hsy++;
                }
            } else {
                detected_objects = yolov8->detectObjects(image);
                for (const struct Object& box : detected_objects) {
                    int class_id = box.label;
                    detected_indices[class_id] = 1;
                    sign_counter[class_id]++;
                    if (sign_counter[class_id] < counter_thresholds[class_id]) {
                        ROS_INFO("%s detected but counter is only %d", class_names[class_id].c_str(), sign_counter[class_id]);
                        continue;
                    }
                    float confidence = box.probability;
                    int x1 = box.rect.x;
                    int y1 = box.rect.y;
                    int x2 = box.rect.x + box.rect.width;
                    int y2 = box.rect.y + box.rect.height;
                    if (populate_sign_msg(sign_msg, image, depthImage, class_id, confidence, x1, y1, x2, y2)) hsy++;
                }
            }

            for (int i = 0; i < OBJECT_COUNT; i++) {
                if (!detected_indices[i]) {
                    sign_counter[i] = 0;
                }
            }

            if(hsy) {
                std_msgs::MultiArrayDimension dim;
                dim.label = "detections";
                dim.size = hsy;
                dim.stride = boxes.size() * 7;
                sign_msg.layout.dim.push_back(dim); 
            }
            // Publish Sign message
            if(publish) pub.publish(sign_msg);
            if(printDuration) {
                stop = high_resolution_clock::now();
                duration = duration_cast<microseconds>(stop - start);
                ROS_INFO("sign durations: %ld", duration.count());
            }
            // for display
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
                    // double maxVal;
                    // double minVal;
                    // if (hasDepthImage) {
                    //     cv::minMaxIdx(depthImage, &minVal, &maxVal);
                    //     depthImage.convertTo(normalizedDepthImage, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
                    // }
                    // yolov8->drawObjectLabels(normalizedDepthImage, detected_objects);
                    // cv::imshow("normalized depth image", normalizedDepthImage);
                    
                    cv::imshow("image", image_copy);
                    cv::waitKey(1);
                }
            }
            if (print) {
                if (ncnn) {
                    for (int i = 0; i < boxes.size(); i++) {
                        double distance = computeMedianDepth(depthImage, boxes[i].x1, boxes[i].y1, boxes[i].x2, boxes[i].y2)/1000;
                        std::cout<< "x1:" << boxes[i].x1<<", y1:"<<boxes[i].y1<<", x2:"<<boxes[i].x2<<", y2:"<<boxes[i].y2
                        <<", conf:"<<boxes[i].score<<", id:"<<boxes[i].cate<<", "<<class_names[boxes[i].cate]<<", dist:"<< distance <<", w:"<<boxes[i].x2-boxes[i].x1<<", h:"<<boxes[i].y2-boxes[i].y1<<std::endl;
                    }
                } else {
                    for (const struct Object& box : detected_objects) {
                        double distance = computeMedianDepth(depthImage, box.rect.x, box.rect.y, box.rect.x + box.rect.width, box.rect.y + box.rect.height)/1000;
                        std::cout<< "x1:" << box.rect.x<<", y1:"<<box.rect.y<<", x2:"<<box.rect.x + box.rect.width<<", y2:"<<box.rect.y + box.rect.height
                        <<", conf:"<<box.probability<<", id:"<<box.label<<", "<<class_names[box.label]<<", dist:"<< distance <<", w:"<<box.rect.width<<", h:"<<box.rect.height<<std::endl;
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
            
            // for (int i = 0; i < croppedDepth.rows; ++i) {
            //     for (int j = 0; j < croppedDepth.cols; ++j) {
            //         double depth = croppedDepth.at<float>(i, j);
            //         if (depth > 100) {  // Only consider valid depth readings
            //             depths.push_back(depth);
            //         }
            //     }
            // }

            if (depths.empty()) {
                return -1; 
            }
            // printf("depths size: %ld, x1: %d, y1: %d, x2: %d, y2: %d, rows: %d, cols: %d\n", depths.size(), x1, y1, x2, y2, croppedDepth.rows, croppedDepth.cols);

            size_t index = depths.size() * 0.5;
            std::nth_element(depths.begin(), depths.begin() + index, depths.end());

            // auto stop = high_resolution_clock::now();
            // auto duration = duration_cast<microseconds>(stop - start);
            // static double avg_duration = 0;
            // static int count = 0;
            // count++;
            // avg_duration = (avg_duration * (count - 1) + duration.count()) / count;
            // printf("computeMedianDepth duration: %ld, avg: %.2f\n", duration.count(), avg_duration);

            if (index % 2) { // if odd
                return depths[index / 2];
            }
            return 0.5 * (depths[(index - 1) / 2] + depths[index / 2]);
        }
};