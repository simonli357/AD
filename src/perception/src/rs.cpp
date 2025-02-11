// realsense_ros_node.cpp
//
// Example ROS node that opens the RealSense device,
// publishes IMU data (gyro and accelerometer) as sensor_msgs/Imu,
// and publishes the RGB and infrared images as sensor_msgs/Image.
//
// Compile this node as part of a ROS package (and link against
// ros, cv_bridge, OpenCV, and librealsense2 libraries).
//
// License: Apache 2.0

#include <ros/ros.h>
#include <sensor_msgs/Imu.h>
#include <sensor_msgs/Image.h>

// For converting OpenCV images to ROS image messages
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>

// Include RealSense Cross Platform API
#include <librealsense2/rs.hpp>

#include <mutex>

int main(int argc, char **argv)
{
    // Initialize the ROS node
    ros::init(argc, argv, "realsense_ros_node");
    ros::NodeHandle nh("~");

    // Create ROS publishers for IMU, color, and infrared images.
    ros::Publisher imu_pub   = nh.advertise<sensor_msgs::Imu>("imu/data", 100);
    ros::Publisher color_pub = nh.advertise<sensor_msgs::Image>("camera/color/image_raw", 10);
    ros::Publisher ir_pub    = nh.advertise<sensor_msgs::Image>("camera/infra/image_raw", 10);

    // Create a RealSense pipeline and configuration
    rs2::pipeline pipe;
    rs2::config cfg;

    // Enable the desired streams:
    //   Color stream: 640x480, BGR8, 30 FPS
    //   Infrared stream: 640x480, Y8 (mono), 30 FPS
    //   Accelerometer stream: Motion data (XYZ 32-bit floats)
    //   Gyro stream: Motion data (XYZ 32-bit floats)
    cfg.enable_stream(RS2_STREAM_COLOR, 640, 480, RS2_FORMAT_BGR8, 30);
    cfg.enable_stream(RS2_STREAM_INFRARED, 640, 480, RS2_FORMAT_Y8, 30);
    cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
    cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);

    // Start streaming with the chosen configuration
    rs2::pipeline_profile profile;
    try
    {
        profile = pipe.start(cfg);
    }
    catch (const rs2::error & e)
    {
        ROS_ERROR("RealSense error calling %s(%s): %s", e.get_failed_function().c_str(),
                  e.get_failed_args().c_str(), e.what());
        return EXIT_FAILURE;
    }
    catch (const std::exception & e)
    {
        ROS_ERROR("Exception: %s", e.what());
        return EXIT_FAILURE;
    }

    // Prepare an IMU message. Since this node does not compute an orientation,
    // we set the orientation_covariance[0] to -1 to indicate that it is not provided.
    sensor_msgs::Imu imu_msg;
    imu_msg.orientation_covariance[0] = -1;  // no orientation estimate available
    imu_msg.angular_velocity_covariance[0] = -1;
    imu_msg.linear_acceleration_covariance[0] = -1;

    // A mutex to protect access to the imu_msg while updating it from different IMU frames.
    std::mutex imu_mutex;

    ros::Rate loop_rate(30); // 30 Hz loop rate

    // Main loop: poll for new frames from the RealSense device.
    while (ros::ok())
    {
        // Poll for a set of available frames (non-blocking)
        rs2::frameset frames;
        if (pipe.poll_for_frames(&frames))
        {
            // --- Process the Color frame ---
            rs2::video_frame color_frame = frames.get_color_frame();
            if (color_frame)
            {
                int width  = color_frame.get_width();
                int height = color_frame.get_height();
                // Construct an OpenCV matrix from the RealSense color frame data.
                cv::Mat color_mat(cv::Size(width, height), CV_8UC3,
                                  (void*)color_frame.get_data(), cv::Mat::AUTO_STEP);
                // Convert the cv::Mat to a ROS image message with the encoding "bgr8"
                std_msgs::Header header;
                header.stamp    = ros::Time::now();
                header.frame_id = "camera_color_optical_frame";
                sensor_msgs::ImagePtr color_msg =
                    cv_bridge::CvImage(header, "bgr8", color_mat).toImageMsg();
                color_pub.publish(color_msg);
            }

            // --- Process the Infrared frame ---
            rs2::video_frame ir_frame = frames.get_infrared_frame();
            if (ir_frame)
            {
                int width  = ir_frame.get_width();
                int height = ir_frame.get_height();
                // Construct a grayscale OpenCV matrix from the infrared frame data.
                cv::Mat ir_mat(cv::Size(width, height), CV_8UC1,
                               (void*)ir_frame.get_data(), cv::Mat::AUTO_STEP);
                std_msgs::Header header;
                header.stamp    = ros::Time::now();
                header.frame_id = "camera_infrared_optical_frame";
                sensor_msgs::ImagePtr ir_msg =
                    cv_bridge::CvImage(header, "mono8", ir_mat).toImageMsg();
                ir_pub.publish(ir_msg);
            }

            // --- Process the Accelerometer frame ---
            rs2::frame accel_frame = frames.first_or_default(RS2_STREAM_ACCEL);
            if (accel_frame)
            {
                rs2::motion_frame motion_accel = accel_frame.as<rs2::motion_frame>();
                if (motion_accel)
                {
                    rs2_vector accel = motion_accel.get_motion_data();
                    {
                        std::lock_guard<std::mutex> lock(imu_mutex);
                        imu_msg.linear_acceleration.x = accel.x;
                        imu_msg.linear_acceleration.y = accel.y;
                        imu_msg.linear_acceleration.z = accel.z;
                        // Set the header stamp and frame_id.
                        imu_msg.header.stamp    = ros::Time::now();
                        imu_msg.header.frame_id = "imu_link";
                    }
                    // Publish the updated IMU message.
                    imu_pub.publish(imu_msg);
                }
            }

            // --- Process the Gyro frame ---
            rs2::frame gyro_frame = frames.first_or_default(RS2_STREAM_GYRO);
            if (gyro_frame)
            {
                rs2::motion_frame motion_gyro = gyro_frame.as<rs2::motion_frame>();
                if (motion_gyro)
                {
                    rs2_vector gyro = motion_gyro.get_motion_data();
                    {
                        std::lock_guard<std::mutex> lock(imu_mutex);
                        imu_msg.angular_velocity.x = gyro.x;
                        imu_msg.angular_velocity.y = gyro.y;
                        imu_msg.angular_velocity.z = gyro.z;
                        imu_msg.header.stamp    = ros::Time::now();
                        imu_msg.header.frame_id = "imu_link";
                    }
                    imu_pub.publish(imu_msg);
                }
            }
        } // if frames are available

        ros::spinOnce();
        loop_rate.sleep();
    } // end while

    // Stop the RealSense pipeline before exiting.
    pipe.stop();
    return EXIT_SUCCESS;
}
