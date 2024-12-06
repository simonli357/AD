#include "ros/ros.h"
#include "yolo-fastestv2.h"
#include "cv_bridge/cv_bridge.h"
#include "image_transport/image_transport.h"
#include "sensor_msgs/image_encodings.h"
#include "std_msgs/Header.h"
#include <chrono>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
using namespace std::chrono;

class signFastest {
    public:
        signFastest(ros::NodeHandle& nh) 
            :it(nh)
        {
            std::cout.precision(4);
            nh.getParam("class_names", class_names);
            // nh.getParam("confidence_thresholds", confidence_thresholds);
            if(!nh.getParam("confidence_thresholds", confidence_thresholds)) {
                ROS_WARN("Failed to get 'confidence_thresholds' parameter.");
            } else {
                std::cout << "confidence_thresholds: " << confidence_thresholds.size() << std::endl;
            }
            if (!nh.getParam("max_distance_thresholds", distance_thresholds)) {
                ROS_WARN("Failed to get 'max_distance_thresholds' parameter.");
            } else {
                for (float val : distance_thresholds) {
                    ROS_INFO("Loaded threshold: %f", val);
                }
                for(int i = 0; i < class_names.size(); i++) {
                    ROS_INFO("class_names: %s, distance_thresholds: %f", class_names[i].c_str(), distance_thresholds[i]);
                }
            }
            nh.getParam("/signFastest2/showFlag", show);
            nh.getParam("/signFastest2/printFlag", print);
            nh.getParam("/signFastest2/printFlag", printDuration); //printDuration
            nh.getParam("/signFastest2/hasDepthImage", hasDepthImage);
            nh.getParam("/signFastest2/real", real);
            std::string model;
            nh.getParam("ncnn_model", model);
            std::cout << "showFlag: " << show << std::endl;
            std::cout << "printFlag: " << print << std::endl;
            std::cout << "printDuration: " << printDuration << std::endl;
            std::cout << "class_names: " << class_names.size() << std::endl;
            std::cout << "confidence_thresholds: " << confidence_thresholds.size() << std::endl;
            std::cout << "distance_thresholds: " << distance_thresholds.size() << std::endl;
            std::cout << "model: " << model << std::endl;
            printf("hasDepthImage: %s\n", hasDepthImage ? "true" : "false");

            // std::string filePathParam = __FILE__;
            // size_t pos = filePathParam.rfind("/") + 1;
            // filePathParam.replace(pos, std::string::npos, "model/sissi753-opt.param");
            // const char* param = filePathParam.c_str();
            // std::string filePathBin = __FILE__;
            // pos = filePathBin.rfind("/") + 1;
            // filePathBin.replace(pos, std::string::npos, "model/sissi753-opt.bin");
            // const char* bin = filePathBin.c_str();
            // api.loadModel(param,bin);

            std::string filePathParam = __FILE__;
            size_t pos = filePathParam.rfind("/") + 1;
            filePathParam.replace(pos, std::string::npos, "model/" + model + ".param");
            const char* param = filePathParam.c_str();

            std::string filePathBin = __FILE__;
            pos = filePathBin.rfind("/") + 1;
            filePathBin.replace(pos, std::string::npos, "model/" + model + ".bin");
            const char* bin = filePathBin.c_str();

            api.loadModel(param, bin);

            pub = nh.advertise<std_msgs::Float32MultiArray>("sign", 10);
            std::cout <<"pub created" << std::endl;
            if(hasDepthImage) {
                std::string topic;
                if (real) {
                    topic = "/camera/aligned_depth_to_color/image_raw";
                } else {    
                    topic = "/camera/depth/image_raw";
                }
                depth_sub = it.subscribe(topic, 3, &signFastest::depthCallback, this);
                std::cout << "depth_sub created" << std::endl;
                ros::topic::waitForMessage<sensor_msgs::Image>(topic, nh);
                std::cout << "got it" << std::endl;
            }
            sub = it.subscribe("/camera/color/image_raw", 3, &signFastest::imageCallback, this);
            // sub = it.subscribe("/camera/image_raw", 3, &signFastest::imageCallback, this);
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
        double distance_makes_sense(double distance, int class_id, float x1, float y1, float x2, float y2) {
            if (distance > distance_thresholds[class_id]) return 0;
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
            // std::cout << "dist:" << distance << ", w:" << width << ", h:" << height << ", expected_dist:" << expected_dist << ", id:" << class_id << std::endl;
            if (distance > expected_dist * 3 || distance < expected_dist * 1/3) {
                ROS_WARN("Distance does not make sense, expected: %.3f, got: %.3f", expected_dist, distance);
                return 0;
            }
            return expected_dist;
        }
        void depthCallback(const sensor_msgs::ImageConstPtr &msg) {
            cv_ptr_depth = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_32FC1);
            if(cv_ptr_depth == nullptr || cv_ptr_depth->image.empty()) {
                ROS_ERROR("cv_bridge failed to convert image");
                return;
            }
        }
        void publishSign(const cv::Mat& image, const cv::Mat& depthImage) {
            if(printDuration) start = high_resolution_clock::now();
            api.detection(image, boxes);
            std_msgs::Float32MultiArray sign_msg;
            sign_msg.layout.data_offset = 0;

            int hsy = 0;
            for (const auto &box : boxes) {
                int class_id = box.cate;
                float confidence = box.score;
                if (confidence >= confidence_thresholds[class_id]) {
                    double distance;
                    if(hasDepthImage) {
                        if (depthImage.empty()) {
                            ROS_ERROR("Depth image is empty");
                            continue;
                        } else {
                            distance = computeMedianDepth(depthImage, box)/1000; // in meters
                        }
                    } else {
                        distance = -1;
                    }
                    double expected_dist = distance_makes_sense(distance, class_id, box.x1, box.y1, box.x2, box.y2);
                    if (!expected_dist) {
                        // ROS_WARN("Distance does not make sense, expected: %.3f, got: %.3f", expected_dist, distance);
                        continue;
                    }
                    sign_msg.data.push_back(box.x1);
                    sign_msg.data.push_back(box.y1);
                    sign_msg.data.push_back(box.x2);
                    sign_msg.data.push_back(box.y2);
                    sign_msg.data.push_back(distance);
                    sign_msg.data.push_back(confidence);
                    sign_msg.data.push_back(static_cast<float>(class_id));
                    hsy++;
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
            pub.publish(sign_msg);
            if(printDuration) {
                stop = high_resolution_clock::now();
                duration = duration_cast<microseconds>(stop - start);
                ROS_INFO("sign durations: %ld", duration.count());
            }
            // for display
            if (show) {
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
                        double distance = computeMedianDepth(depthImage, boxes[i])/1000; 
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
            }
            if (print) {
                for (int i = 0; i < boxes.size(); i++) {
                    double distance = computeMedianDepth(depthImage, boxes[i])/1000;
                    std::cout<< "x1:" << boxes[i].x1<<", y1:"<<boxes[i].y1<<", x2:"<<boxes[i].x2<<", y2:"<<boxes[i].y2
                     <<", conf:"<<boxes[i].score<<", id:"<<boxes[i].cate<<", "<<class_names[boxes[i].cate]<<", dist:"<< distance <<", w:"<<boxes[i].x2-boxes[i].x1<<", h:"<<boxes[i].y2-boxes[i].y1<<std::endl;
                }
            }
        }
        void imageCallback(const sensor_msgs::ImageConstPtr &msg) {
            if(printDuration) start = high_resolution_clock::now();
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            if (!cv_ptr) {
                ROS_ERROR("cv_bridge failed to convert image");
                return;
            }
            publishSign(cv_ptr->image, cv_ptr_depth->image);
        }
        
    private:
        yoloFastestv2 api;
        image_transport::Subscriber sub;
        image_transport::Subscriber depth_sub;
        image_transport::ImageTransport it;
        ros::Publisher pub;
        bool show;
        bool print;
        bool printDuration;
        bool hasDepthImage;
        bool real;
        cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImagePtr cv_ptr_depth = nullptr;
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
        std::vector<std::string> class_names;

        static constexpr double SIGN_H2D_RATIO = 31.57;
        static constexpr double LIGHT_W2D_RATIO = 41.87;
        static constexpr double CAR_H2D_RATIO = 90.15;

        double computeMedianDepth(const cv::Mat& depthImage, const TargetBox& box) {
            // Ensure the bounding box coordinates are valid
            int x1 = std::max(0, box.x1);
            int y1 = std::max(0, box.y1);
            int x2 = std::min(depthImage.cols, box.x2);
            int y2 = std::min(depthImage.rows, box.y2);
            croppedDepth = depthImage(cv::Rect(x1, y1, x2 - x1, y2 - y1));
            std::vector<double> depths;
            for (int i = 0; i < croppedDepth.rows; ++i) {
                for (int j = 0; j < croppedDepth.cols; ++j) {
                    double depth = croppedDepth.at<float>(i, j);
                    if (depth > 100) {  // Only consider valid depth readings
                        depths.push_back(depth);
                    }
                }
            }
            if (depths.empty()) {
                return -1; 
            }
            // Find the median using std::nth_element
            size_t index20Percent = depths.size() * 0.1;
            std::nth_element(depths.begin(), depths.begin() + index20Percent, depths.end());
            if (index20Percent % 2) { // if odd
                return depths[index20Percent / 2];
            }
            return 0.5 * (depths[(index20Percent - 1) / 2] + depths[index20Percent / 2]);
        }
};

int main(int argc, char **argv) {
    std::cout << "OpenCV version : " << CV_VERSION << std::endl;
    int opt;
    
    // Initialize ROS node and publisher
    ros::init(argc, argv, "object_detector");
    ros::NodeHandle nh;
    
    signFastest signFastest(nh);
    //define rate
    ros::Rate loop_rate(25);

    // Spin ROS node
    while(ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}