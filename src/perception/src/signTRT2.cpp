#include "ros/ros.h"
#include "yolo-fastestv2.h"
#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "image_transport/image_transport.h"
#include "cv_bridge/cv_bridge.h"
#include "std_msgs/Header.h"
#include "engine.h"
#include <chrono>
#include <vector>
#include <std_msgs/Float32MultiArray.h>
#include "yolov8.h"
#include <iostream>
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
using namespace nvinfer1;
// Options options;
// Engine model(options);
//get the model path
std::string model_name = "citycocov2lgtclab_20"; 
// std::string model_name = "v2originalTRT"; 
std::string current_path = getSourceDirectory();
std::string modelPath = current_path + "/model/" + model_name + ".onnx";
YoloV8Config config;
YoloV8 yolov8 = YoloV8(modelPath, config);


class signTRT{
    public:
        
        // Specify what precision to use for inference
        // FP16 is approximately twice as fast as FP32.
        
        signTRT(ros::NodeHandle& nh):it(nh),reshapedOutput(1,std::vector<std::vector<float>>(8400, std::vector<float>(17)))
        // scores(10),classes(10),tmp_boxes(10)//, inputDims(inputDims)
        {
            
            //loading from yaml file
            nh.getParam("class_names",class_names);
            nh.getParam("confidence_thresholds", confidence_thresholds);
            nh.getParam("max_distance_thresholds", distance_thresholds);
            nh.getParam("/showFlag",show);
            nh.getParam("/printFlag",print);
            nh.getParam("/printDuration",printDuration);
            
            std::cout << "showFlag: " << show << std::endl;
            std::cout << "printFlag: " << print << std::endl;
            std::cout << "printDuration: " << printDuration << std::endl;
            std::cout << "class_names: " << class_names.size() << std::endl;
            std::cout << "confidence_thresholds: " << confidence_thresholds.size() << std::endl;
            // std::cout << "distance_thresholds: " << distance_thresholds.size() << std::endl;
            std::cout << "model: " << model_name << std::endl;
            inputDims = {640,640};
            //topics management
            pub = nh.advertise<std_msgs::Float32MultiArray>("sign",10);
            std::cout << "pub created" << std::endl;
            
            depth_sub = it.subscribe("/camera/depth/image_raw",3,&signTRT::depthCallback,this);
            std::cout << "waiting for depth image" << std::endl;
            ros::topic::waitForMessage<sensor_msgs::Image>("/camera/depth/image_raw",nh);
            std::cout << "got depth image" << std::endl;
            sub = it.subscribe("/camera/color/image_raw",3,&signTRT::imageCallback,this);

        }
        void depthCallback(const sensor_msgs::ImageConstPtr& msg){
            try{
                cv_ptr_depth = cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::TYPE_32FC1);
            }
            catch(cv_bridge::Exception& e) {
                ROS_ERROR("cv_bridge exception: %s",e.what());
                return;
            }
        }
        void imageCallback(const sensor_msgs::ImageConstPtr& msg){
            if(printDuration) start = std::chrono::high_resolution_clock::now();
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            std::vector<Object> detected_objects = yolov8.detectObjects(cv_ptr->image);
            // std::cout << "Avg Inference time: " << (t2) / 1000.f << " ms" << std::endl;
            // preciseStopwatch s3;

            int hsy = 0;
           
            std_msgs::Float32MultiArray sign_msg;
            sign_msg.layout.data_offset = 0;
            for (const struct Object& box : detected_objects) {
                int class_id = box.label;
                float confidence = box.probability;
                if (confidence >= confidence_thresholds[class_id]){
                    double distance = computeMedianDepth(cv_ptr_depth->image,box)/1000;
                    // double distance = 0.7;
                    if (distance <= distance_thresholds[class_id]){
                        sign_msg.data.push_back(box.rect.x);
                        sign_msg.data.push_back(box.rect.y);
                        sign_msg.data.push_back(box.rect.x + box.rect.width);
                        sign_msg.data.push_back(box.rect.y + box.rect.height);
                        sign_msg.data.push_back(distance);
                        sign_msg.data.push_back(confidence);
                        sign_msg.data.push_back(static_cast<float>(class_id));
                        hsy++;
                    }
                }
            }

            if(hsy) {
                std_msgs::MultiArrayDimension dim;
                dim.label = "detections";
                dim.size = hsy;
                dim.stride = boxes.size() * 7;
                sign_msg.layout.dim.push_back(dim); 
            }
            //publish message
            pub.publish(sign_msg);
            if (printDuration) {
                stop = std::chrono::high_resolution_clock::now();
                duration = std::chrono::duration_cast<std::chrono::microseconds>(stop-start);
                ROS_INFO("sign durations: %ld", duration.count());
            }

            //display
            if(show) {
                std::vector<double> distances;
                yolov8.drawObjectLabels(cv_ptr->image, detected_objects, distances);
                double maxVal;
                double minVal;
                cv::minMaxIdx(cv_ptr_depth->image,&minVal,&maxVal);
                cv_ptr_depth->image.convertTo(normalizedDepthImage,CV_8U,255.0/(maxVal-minVal),-minVal*255.0/(maxVal-minVal));
                yolov8.drawObjectLabels(normalizedDepthImage, detected_objects, distances);

                cv::imshow("normalized depth image", normalizedDepthImage);
                cv::imshow("image", cv_ptr->image);
                cv::waitKey(1);
            }
            if (print) {
            for (int i = 0; i < detected_objects.size(); i++) {
                double distance = computeMedianDepth(cv_ptr_depth->image, detected_objects[i]) / 1000;
                std::cout << "x1:" << detected_objects[i].rect.x << ", y1:" << detected_objects[i].rect.y << ", x2:" << detected_objects[i].rect.x + detected_objects[i].rect.width << ", y2:" << detected_objects[i].rect.y + detected_objects[i].rect.height
                << ", conf:" << detected_objects[i].probability << ", id:" << detected_objects[i].label << ", " << class_names[detected_objects[i].label] << ", dist:" << distance << std::endl;

            }
            }
        }
        
    private:
    //topic, sub and pub param
        image_transport::Subscriber sub;
        image_transport::Subscriber depth_sub;
        image_transport::ImageTransport it;
        ros::Publisher pub;
        bool show;
        bool print;
        bool printDuration;
        cv_bridge::CvImagePtr cv_ptr;
        cv_bridge::CvImagePtr cv_ptr_depth;
        cv::Mat normalizedDepthImage;
        cv::Mat croppedDepth;
        // std_msgs::Float32MultiArray sign_msg;
    //results
        std::vector<TargetBox> boxes;
        std::vector<TargetBox> boxes_depth;
        std::chrono::high_resolution_clock::time_point start;
        std::chrono::high_resolution_clock::time_point stop;
        std::chrono::microseconds duration;

        std::vector<double> depths;
        std::vector<float> confidence_thresholds;
        std::vector<float> distance_thresholds;
        std::vector<std::string> class_names;

        std::vector<cv::Rect> tmp_boxes;
        std::vector<float> scores;
        std::vector<int> classes;
        TargetBox tmp_box;


        std::vector<int> indices;

        size_t class_ind;
        float maxscore;
        std::vector<float>::iterator maxscore_index;
        float m_imgHeight;
        float m_imgWidth;
        float m_ratio;
    
    //engine inference related
        // YoloV8 yoloV8;
        std::vector<std::vector<std::vector<float>>> featureVectors; //output memory
        std::vector<std::vector<std::vector<float>>> reshapedOutput; //1,8400,17
        std::vector<std::vector<cv::cuda::GpuMat>> inputs; //input memory
        cv::cuda::GpuMat img;
        // std::vector<nvinfer1::Dims3>& inputDims;
        std::vector<int> inputDims;
        std::vector<nvinfer1::Dims> outputDims;
        //auto& inputDims;
        bool succ;
        size_t batchSize;
        cv::cuda::GpuMat resized;
        std::vector<cv::cuda::GpuMat> input;
        std::vector<float> featureVector;

        double computeMedianDepth(const cv::Mat& depthImage, const struct Object& box) {
            
            // Ensure the bounding box coordinates are valid
            int x1 = std::max(0, static_cast<int>(box.rect.x));
            int y1 = std::max(0, static_cast<int>(box.rect.y));
            int x2 = std::min(depthImage.cols, static_cast<int>((box.rect.x) + (box.rect.width)));
            // int y2 = std::min(depthImage.rows, *box.rect.y + *box.rect.height);
            int y2 = std::min(depthImage.rows, static_cast<int>(box.rect.y + box.rect.height));
            
            // Crop the depth image to the bounding box
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
            // std::cout << "compute3" <<std::endl;
            std::sort(depths.begin(), depths.end());

            //take the closest 20% of the pixels
            size_t index20Percent = depths.size() * 0.2;
            if (index20Percent <= 0) { // if there are less than 20% valid pixels
            return depths[0];
            }
            if (index20Percent % 2) { // if odd
            return depths[index20Percent / 2];
            }
            return 0.5 * (depths[(index20Percent - 1) / 2] + depths[index20Percent / 2]);
        } 


};

int main(int argc, char** argv) {
  int opt;
  ROS_INFO("model path: %s", modelPath.c_str());

  // Initialize ROS node and publisher
    ros::init(argc, argv, "object_detector");
  ros::NodeHandle nh;
  
    std::cout << "ck0" << std::endl;
  signTRT signTRT(nh);
  //define rate
  ros::Rate loop_rate(25);

  // Spin ROS node
  while (ros::ok()) {
    ros::spinOnce();
    loop_rate.sleep();
  }

  return 0;
}