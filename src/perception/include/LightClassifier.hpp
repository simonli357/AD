#pragma once

#include <opencv2/opencv.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/core.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/cudaimgproc.hpp>
#include "utils/constants.h"
#include "utils/helper.h"
#include <iostream>

using namespace std::chrono;
using namespace VehicleConstants;

class LightClassifier {
    public:
        LightClassifier() {}

        ~LightClassifier() {}

        bool show = false;
        cv::Mat hsv;
        cv::Mat value_channel;
        cv::Mat bright_mask;
        std::vector<std::vector<cv::Point>> contours;
        cv::Mat detected_with_contours;
        // HSV Ranges for Toy Lights (adjust based on your hardware)
        cv::Scalar red_low1 = cv::Scalar(0, 150, 150);
        cv::Scalar red_high1 = cv::Scalar(10, 255, 255);
        cv::Scalar red_low2 = cv::Scalar(170, 150, 150);
        cv::Scalar red_high2 = cv::Scalar(180, 255, 255);
        cv::Scalar yellow_low = cv::Scalar(22, 150, 150);
        cv::Scalar yellow_high = cv::Scalar(28, 255, 255);
        cv::Scalar green_low = cv::Scalar(60, 150, 150);
        cv::Scalar green_high = cv::Scalar(85, 255, 255);

        cv::Mat red_mask, yellow_mask, green_mask;

        float getAdaptiveThreshold(float baseline_brightness, float min_val, float max_val) {
            return min_val + (max_val - min_val) * (1.0f - baseline_brightness);
        }
    
        void updateThresholds(float baseline_brightness) {
            // Saturation: Higher threshold in bright conditions
            float sat_threshold = getAdaptiveThreshold(baseline_brightness, 100.0f, 200.0f);
            
            // Value: Lower threshold in dark conditions
            float val_threshold = getAdaptiveThreshold(baseline_brightness, 50.0f, 150.0f);
            
            // Update HSV ranges
            red_low1 = cv::Scalar(0, sat_threshold, val_threshold);
            red_high1 = cv::Scalar(10, 255, 255);
            red_low2 = cv::Scalar(170, sat_threshold, val_threshold);
            red_high2 = cv::Scalar(180, 255, 255);
            yellow_low = cv::Scalar(22, sat_threshold, val_threshold);
            yellow_high = cv::Scalar(28, 255, 255);
            green_low = cv::Scalar(60, sat_threshold, val_threshold);
            green_high = cv::Scalar(85, 255, 255);
        }

        LightColor classify(cv::Mat detected_light) {
            // preprocessing
            cv::cvtColor(detected_light, hsv, cv::COLOR_BGR2HSV);
            cv::GaussianBlur(hsv, hsv, cv::Size(5,5), 0);
        
            // adaptive Brightness Detection
            cv::extractChannel(hsv, value_channel, 2);
            float baseline_brightness = cv::mean(value_channel)[0] / 255.0f;
            updateThresholds(baseline_brightness);
            cv::threshold(value_channel, bright_mask, 0, 255, 
                         cv::THRESH_BINARY | cv::THRESH_OTSU);
        
            // contour Detection with Size Filtering
            contours.clear();
            cv::findContours(bright_mask, contours, cv::RETR_EXTERNAL, 
                            cv::CHAIN_APPROX_SIMPLE);

            const int img_height = detected_light.rows;
            const int img_width = detected_light.cols;
            const int circle_diameter = (img_width * 0.5);
            const int empirical_y_center_red = circle_diameter/2 + img_height/22;
            const int empirical_y_center_yellow = empirical_y_center_red + circle_diameter;
            const int empirical_y_center_green = empirical_y_center_yellow + circle_diameter;
            const int empirical_x_center = img_width/2;
            
            if (show) {
                cv::imshow("hsv", hsv);
                cv::waitKey(1);
                cv::imshow("bright_mask", bright_mask);
                cv::waitKey(1);
                // draw the contours on the image and then show
                detected_with_contours = detected_light.clone();
                cv::drawContours(detected_with_contours, contours, -1, cv::Scalar(255, 0, 0), 2);
                cv::imshow("Contours", detected_with_contours);
                std::cout << "num contours: " << contours.size() << std::endl;
                //draw 3 circles of diameter circle_diameter, starting from top of bbox
                cv::circle(detected_with_contours, cv::Point(img_width/2,  empirical_y_center_red), circle_diameter/2, cv::Scalar(0, 0, 255), 2);
                cv::circle(detected_with_contours, cv::Point(img_width/2,  empirical_y_center_yellow), circle_diameter/2, cv::Scalar(0, 0, 255), 2);
                cv::circle(detected_with_contours, cv::Point(img_width/2,  empirical_y_center_green), circle_diameter/2, cv::Scalar(0, 0, 255), 2);
            }
            
            LightColor position_estimated_color = LightColor::UNDETERMINED;
            for (const auto& contour : contours) {
                cv::Rect bbox = cv::boundingRect(contour);
                
                // Validate bounding box
                if (bbox.width <= 0 || bbox.height <= 0) continue;
                if (bbox.x + bbox.width > img_width || bbox.y + bbox.height > img_height) {
                    // std::cout << "contour out of bounds" << std::endl;
                    continue;
                }

                // Circularity check
                float area = cv::contourArea(contour);
                float perimeter = cv::arcLength(contour, true);
                if (perimeter <= 0) continue;
                float circularity = (4 * CV_PI * area) / (perimeter * perimeter);
                const float MIN_CIRCULARITY = 0.357f;
                if (circularity < MIN_CIRCULARITY) {
                    // std::cout << "not circular: " << circularity << std::endl;
                    continue;
                }
                
                // size filter (8-40% of bounding box height)
                double height_norm = std::sqrt(area) / detected_light.rows;
                if (height_norm < 0.08 || height_norm > 0.4) {
                    // std::cout << "too small or too big: " << height_norm << std::endl;
                    continue;
                }

                if (show) cv::drawContours(detected_with_contours, std::vector{contour}, -1, cv::Scalar(0, 255, 0), 2);
        
                // Position scoring (normalized vertical position)
                cv::Moments m = cv::moments(contour);
                float y_center = static_cast<float>(m.m01/m.m00);
                float x_center = static_cast<float>(m.m10/m.m00);
                //check if it's too far from img center
                if (std::abs(x_center - img_width/2) > img_width/5) {
                    // std::cout << "too far from center: xcenter: " << x_center << ", img_width/2: " << img_width/2 << std::endl;
                    continue;
                }
                // float circle_diameter = (bbox.width + img_width * 0.75)/2;
                float dist_to_red = std::abs(y_center - empirical_y_center_red);
                float dist_to_yellow = std::abs(y_center - empirical_y_center_yellow);
                float dist_to_green = std::abs(y_center - empirical_y_center_green);
                if (dist_to_red < dist_to_yellow && dist_to_red < dist_to_green) {
                    position_estimated_color = LightColor::RED;
                    // std::cout << "closest to red. dists: " << dist_to_red << ", " << dist_to_yellow << ", " << dist_to_green << std::endl;
                    break;
                } else if (dist_to_yellow < dist_to_red && dist_to_yellow < dist_to_green) {
                    position_estimated_color = LightColor::YELLOW;
                    // std::cout << "closest to yellow. dists: " << dist_to_red << ", " << dist_to_yellow << ", " << dist_to_green << std::endl;
                    break;
                } else {
                    position_estimated_color = LightColor::GREEN;
                    // std::cout << "closest to green. dists: " << dist_to_red << ", " << dist_to_yellow << ", " << dist_to_green << std::endl;
                    break;
                }
            }
            
            if (show) {
                cv::imshow("Detected with Contours", detected_with_contours);
                cv::waitKey(1);
            }

            if (position_estimated_color != LightColor::UNDETERMINED) {
                return position_estimated_color;
            }
            red_mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
            yellow_mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);
            green_mask = cv::Mat::zeros(img_height, img_width, CV_8UC1);

            cv::circle(red_mask, cv::Point(empirical_x_center, empirical_y_center_red), circle_diameter/2, 255, -1);
            cv::circle(yellow_mask, cv::Point(empirical_x_center, empirical_y_center_yellow), circle_diameter/2, 255, -1);
            cv::circle(green_mask, cv::Point(empirical_x_center, empirical_y_center_green), circle_diameter/2, 255, -1);

            // compute mean HSV for each circle
            cv::Scalar mean_red = cv::mean(hsv, red_mask);
            cv::Scalar mean_yellow = cv::mean(hsv, yellow_mask);
            cv::Scalar mean_green = cv::mean(hsv, green_mask);
            
            // brightness score
            float adaptive_threshold = std::max(baseline_brightness * 1.5f * 0.573f, 0.2f);
            float brightness_score_red = mean_red[2] / 255.0f;
            float brightness_score_yellow = mean_yellow[2] / 255.0f;
            float brightness_score_green = mean_green[2] / 255.0f;
            if (brightness_score_red > adaptive_threshold && brightness_score_red > brightness_score_yellow && brightness_score_red > brightness_score_green) {
                return LightColor::RED;
            } else if (brightness_score_yellow > adaptive_threshold && brightness_score_yellow > brightness_score_red && brightness_score_yellow > brightness_score_green) {
                return LightColor::YELLOW;
            } else if (brightness_score_green > adaptive_threshold && brightness_score_green > brightness_score_red && brightness_score_green > brightness_score_yellow) {
                return LightColor::GREEN;
            }
            // std::cout << "brightness red: " << brightness_score_red << ", yellow: " << brightness_score_yellow << ", green: " << brightness_score_green << ", threshold: " << adaptive_threshold << std::endl;
            return classify_backup(detected_light);
        }

        LightColor classify_backup(cv::Mat detected_light) {
            // Convert to grayscale and HSV
            cv::Mat gray_image, hsv_image, bright_mask;
            cv::cvtColor(detected_light, gray_image, cv::COLOR_BGR2GRAY);
            cv::cvtColor(detected_light, hsv_image, cv::COLOR_BGR2HSV);

            // Threshold for brightness
            cv::threshold(gray_image, bright_mask, 200, 255, cv::THRESH_BINARY);

            // Find contours in the bright mask
            std::vector<std::vector<cv::Point>> contours;
            cv::findContours(bright_mask, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

            // Iterate through contours to find circular regions
            for (const auto& contour : contours) {
                // if (isCircular(contour)) {
                if (true) {
                    // Get bounding box of the circular region
                    cv::Rect circle_rect = cv::boundingRect(contour);
                    cv::Mat circle_region = hsv_image(circle_rect);

                    cv::Mat detected_with_circle = detected_light.clone();
                    cv::rectangle(detected_with_circle, circle_rect, cv::Scalar(0, 255, 0), 2);

                    // Define HSV ranges for red, green, yellow
                    cv::Scalar lower_red1(0, 50, 50), upper_red1(10, 255, 255);
                    cv::Scalar lower_red2(170, 50, 50), upper_red2(180, 255, 255);
                    cv::Scalar lower_green(40, 50, 50), upper_green(90, 255, 255);
                    cv::Scalar lower_yellow(15, 50, 50), upper_yellow(35, 255, 255);

                    // Create masks for each color
                    cv::Mat mask_red1, mask_red2, red_mask, green_mask, yellow_mask;
                    cv::inRange(circle_region, lower_red1, upper_red1, mask_red1);
                    cv::inRange(circle_region, lower_red2, upper_red2, mask_red2);
                    cv::bitwise_or(mask_red1, mask_red2, red_mask);

                    cv::inRange(circle_region, lower_green, upper_green, green_mask);
                    cv::inRange(circle_region, lower_yellow, upper_yellow, yellow_mask);

                    // Calculate the intensity of each color
                    double red_area = cv::countNonZero(red_mask);
                    double green_area = cv::countNonZero(green_mask);
                    double yellow_area = cv::countNonZero(yellow_mask);
                    double total_area = circle_region.rows * circle_region.cols;

                    double red_ratio = red_area / total_area;
                    double green_ratio = green_area / total_area;
                    double yellow_ratio = yellow_area / total_area;

                    // std::cout << "Red Ratio: " << red_ratio << ", Green Ratio: " << green_ratio
                    //   << ", Yellow Ratio: " << yellow_ratio << std::endl;
                    if (red_ratio > green_ratio && red_ratio > yellow_ratio && red_ratio > 0.2) {
                        return LightColor::RED;
                    } else if (green_ratio > red_ratio && green_ratio > yellow_ratio && green_ratio > 0.2) {
                        return LightColor::GREEN;
                    } else if (yellow_ratio > red_ratio && yellow_ratio > green_ratio && yellow_ratio > 0.2) {
                        return LightColor::YELLOW;
                    }
                }
            }
            return LightColor::UNDETERMINED;
        }
        
};
