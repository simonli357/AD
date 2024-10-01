#include <opencv2/opencv.hpp>
#include "yolo-fastestv2.h"

int main(int argc, char **argv) {
    std::vector<std::string> class_names = {
        "oneway", "highwayentrance", "stopsign", "roundabout", "park", 
        "crosswalk", "noentry", "highwayexit", "priority", "lights", 
        "block", "pedestrian", "car"
    };

    // Check for video file argument
    if (argc != 2) {
        std::cerr << "Usage: " << argv[0] << " <VideoPath>\n";
        return -1;
    }

    // Open the video file
    cv::VideoCapture cap(argv[1]);
    if (!cap.isOpened()) {
        std::cerr << "Error opening video file\n";
        return -1;
    }

    // Create an object of YOLO Fastest
    yoloFastestv2 detector;
    detector.loadModel("/home/simonli/Simulator/src/Control/src/model/sissi753-opt.param", "/home/simonli/Simulator/src/Control/src/model/sissi753-opt.bin");

    int originalHeight = 1080;
    int newWidth = static_cast<int>(originalHeight * 4/3);

    // Video writer
    cv::VideoWriter writer;
    int codec = cv::VideoWriter::fourcc('M', 'J', 'P', 'G');
    // double fps = 25.0;
    double fps = cap.get(cv::CAP_PROP_FPS);
    cv::Size frame_size(cap.get(cv::CAP_PROP_FRAME_WIDTH), cap.get(cv::CAP_PROP_FRAME_HEIGHT));
    std::cout << "shape: " << frame_size << ", fps: " << fps << std::endl;

    cv::Size new_frame_size(newWidth, originalHeight);
    writer.open("output4.avi", codec, fps, frame_size, true);
    // writer.open("output4.avi", codec, fps, new_frame_size, true);

    if (!writer.isOpened()) {
        std::cerr << "Could not open the output video file for write\n";
        return -1;
    }

    // Define colors for different classes
    std::vector<cv::Scalar> colors = {cv::Scalar(255, 0, 0), cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
                                      cv::Scalar(255, 255, 0), cv::Scalar(0, 255, 255)};

    long totalFrames = static_cast<long>(cap.get(cv::CAP_PROP_FRAME_COUNT));
    long currentFrame = 0;

    cv::Mat frame, croppedFrame;
    while (true) {
        cap >> frame;
        if (frame.empty()) {
            break;
        }

        // cv::Rect cropRegion(0, 0, newWidth, originalHeight); // Cropping from the left
        // croppedFrame = frame(cropRegion);
        croppedFrame = frame;

        std::vector<TargetBox> boxes;
        detector.detection(croppedFrame, boxes);

        // Process and display the boxes
        for (const auto &box : boxes) {
            // std::cout << box.x1 << ", " << box.y1 << ", " << box.x2 << ", " << box.y2 << ", " << box.score << ", " << box.cate << std::endl;
            int class_id = box.cate;
            if(box.score < 0.753) continue;
            std::string label = class_names[class_id] + ": " + std::to_string(static_cast<int>(box.score * 100)) + "%";

            // Get text size for background
            int baseline;
            cv::Size label_size = cv::getTextSize(label, cv::FONT_HERSHEY_SIMPLEX, 0.6, 1, &baseline);

            // Draw rectangle and label background
            cv::rectangle(croppedFrame, cv::Point(box.x1, box.y1), cv::Point(box.x2, box.y2), cv::Scalar(0, 255, 0), 2);
            cv::rectangle(croppedFrame, cv::Point(box.x1, box.y1 - label_size.height - baseline), 
                          cv::Point(box.x1 + label_size.width, box.y1), cv::Scalar(0, 255, 0), cv::FILLED);

            // Put text on the frame
            cv::putText(croppedFrame, label, cv::Point(box.x1, box.y1 - baseline), 
                        cv::FONT_HERSHEY_SIMPLEX, 1, cv::Scalar(0, 0, 0), 1);
        }

        writer.write(croppedFrame);
        currentFrame++;
        float progress = (currentFrame / static_cast<float>(totalFrames)) * 100.0f;
        std::cout << "Progress: " << progress << "%\r"; // '\r' returns the cursor to the start of the line
        std::cout.flush();
    }

    cap.release();
    writer.release();
    return 0;
}