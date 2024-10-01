#include "ros/ros.h"
#include "utils/localisation.h"
#include "geometry_msgs/PoseWithCovarianceStamped.h"
#include "std_msgs/Header.h"
#include "tf2/LinearMath/Quaternion.h"
#include <tf2_geometry_msgs/tf2_geometry_msgs.h>
#include <std_msgs/Float64MultiArray.h>
#include <deque>
#include <sensor_msgs/Imu.h>

class LocalizationBridge {
    public:
        LocalizationBridge(ros::NodeHandle& nh_, std::string name) :
            nh(nh_), robot_name(name)
        {
            nh.getParam("/x_offset", x_offset);
            nh.getParam("/y_offset", y_offset);
            nh.getParam("/max_noise", max_noise);
            nh.getParam("/cov", cov);
            nh.getParam("/manual_integration", manual);
            ROS_INFO("manual_integration: %d", manual);
            pose_pub = nh_.advertise<geometry_msgs::PoseWithCovarianceStamped>("/gps", 10);
            std::string gps_topic = "/" + robot_name + "/localisation";
            sub = nh_.subscribe(gps_topic, 10, &LocalizationBridge::localisationCallback, this);
            commands_sub = nh_.subscribe("/commands", 10, &LocalizationBridge::commandsCallback, this);
            imu_sub = nh_.subscribe("/realsense/imu", 10, &LocalizationBridge::imuCallback, this);
            std::fill(std::begin(msg.pose.covariance), std::end(msg.pose.covariance), 0.0);
            // σ^2 = (b - a)^2 / 12
            double pos_cov = std::pow(2*max_noise, 2) / 12;
            ROS_INFO("Position covariance: %f", pos_cov);
            if(manual) pos_cov += cov;
            ROS_INFO("new Position covariance: %f", pos_cov);
            msg.pose.covariance[0] = pos_cov; // Variance for posA
            msg.pose.covariance[7] = pos_cov; // Variance for posB
            msg.pose.covariance[14] = pos_cov; 
            msg.pose.covariance[21] = pos_cov; 
            msg.pose.covariance[28] = pos_cov; 
            msg.pose.covariance[35] = pos_cov;
            msg.header.frame_id = "odom";
            wheelbase = 0.27;
            prev_time = 0.0;
            prev_yaw = M_PI / 2;
            prev_v = 0.0;
            prev_steering = 0;
        }
    private:
        std::string robot_name;
        ros::NodeHandle nh;
        ros::Publisher pose_pub;
        ros::Subscriber sub;
        ros::Subscriber commands_sub;
        ros::Subscriber imu_sub;
        double max_noise, wheelbase, cov;
        double prev_time, prev_yaw, prev_v, prev_steering, dx, dy, imu_yaw, x_offset, y_offset;
        bool manual;
        // Create a PoseWithCovarianceStamped message
        geometry_msgs::PoseWithCovarianceStamped msg;
        std::deque<std_msgs::Float64MultiArray> commands;

        void localisationCallback(const utils::localisation::ConstPtr& msg_in) {
            // auto start = std::chrono::high_resolution_clock::now();

            // Copy the header
            msg.header.stamp = msg_in->header.stamp; // localisation msg is delayed by 1s

            double x0 = msg_in->posA + x_offset;
            double y0 = msg_in->posB + y_offset;
            double time0 = msg_in->header.stamp.toSec();
            prev_time = time0;
            
            if (manual) {
                for (auto it = commands.begin(); it != commands.end();) {
                    double time = it->data[3];
                    if (time > time0) {
                        double dt = time - prev_time;
                        if(dt > 0.0) {
                            ROS_INFO("dt: %.3f, yaw: %.3f, v: %.3f, steering: %.3f", dt, prev_yaw, prev_v, prev_steering);
                            updateStatesRK4(prev_yaw, prev_v, prev_steering, dt);
                            // updateStatesRK4(prev_yaw, prev_v, prev_steering, dt);
                            prev_time = time;
                            x0 += dx;
                            y0 += dy;
                            prev_steering = it->data[0];
                            prev_v = it->data[1];
                            prev_yaw = it->data[2];
                        } else {
                            ROS_WARN("dt is negative: %f", dt);
                        }
                        ++it;
                    } else {
                        it = commands.erase(it);
                    }
                }

                double time_now = ros::Time::now().toSec();
                double dt = time_now - prev_time;
                if(dt > 0.0) {
                    ROS_INFO("dt: %.3f, yaw: %.3f, v: %.3f, steering: %.3f", dt, prev_yaw, prev_v, prev_steering);
                    updateStatesRK4(prev_yaw, prev_v, prev_steering, dt);
                    // updateStatesRK4(imu_yaw, prev_v, prev_steering, dt);
                    prev_time = time_now;
                    x0 += dx;
                    y0 += dy;
                } else {
                    ROS_WARN("dt is negative: %f", dt);
                }
            }

            msg.pose.pose.position.x = x0;
            msg.pose.pose.position.y = y0;
            msg.pose.pose.position.z = 0.0; // 2d

            // ROS_INFO("x: %3.3f, y: %3.3f", x0, y0);

            // tf2::Quaternion q;
            // q.setRPY(0, 0, msg_in->rotA); 
            // msg.pose.pose.orientation = tf2::toMsg(q);

            // Publish the message
            pose_pub.publish(msg);
            // auto end = std::chrono::high_resolution_clock::now();
            // auto duration = std::chrono::duration_cast<std::chrono::microseconds>(end-start);
            // ROS_INFO("Time taken: %3.3f ms", duration.count()/1000.0); 
        }
        void imuCallback(const sensor_msgs::Imu::ConstPtr& msg_in) {
            tf2::Quaternion q;
            tf2::fromMsg(msg_in->orientation, q);
            double roll, pitch, yaw;
            tf2::Matrix3x3(q).getRPY(roll, pitch, yaw);
            imu_yaw = yaw;
        }
        void commandsCallback(const std_msgs::Float64MultiArray::ConstPtr& msg_in) {
            // msg contains [steering, v, yaw, timestamp]
            if (msg_in->data.size() >= 4) {
                std_msgs::Float64MultiArray msg_out = *msg_in;
                double time_now = ros::Time::now().toSec();
                msg_out.data[3] = time_now;
                commands.push_back(msg_out);
                msg_out.data[2] = imu_yaw;
            }
            // double time = msg_in->data[3];
            // ROS_INFO("time_now: %3.3f, time: %3.3f, dt: %3.3f", time_now, time, time_now - time);
        }
        void updateStatesRK4(double yaw, double v, double steering, double dt) {
            double magnitude = v * dt;
            double yaw_rate = magnitude * tan(-steering * M_PI / 180) / wheelbase;
            
            double k1_x = magnitude * cos(yaw);
            double k1_y = magnitude * sin(yaw);
            double k1_yaw = yaw_rate;

            double k2_x = magnitude * cos(yaw + dt / 2 * k1_yaw);
            double k2_y = magnitude * sin(yaw + dt / 2 * k1_yaw);
            double k2_yaw = yaw_rate;

            double k3_x = magnitude * cos(yaw + dt / 2 * k2_yaw);
            double k3_y = magnitude * sin(yaw + dt / 2 * k2_yaw);
            double k3_yaw = yaw_rate;

            double k4_x = magnitude * cos(yaw + dt * k3_yaw);
            double k4_y = magnitude * sin(yaw + dt * k3_yaw);
            double k4_yaw = yaw_rate;

            dx = 1 / 6.0 * (k1_x + 2 * k2_x + 2 * k3_x + k4_x);
            dy = 1 / 6.0 * (k1_y + 2 * k2_y + 2 * k3_y + k4_y);
        }
};

int main(int argc, char **argv) {
    // Initialize the ROS node
    ros::init(argc, argv, "localization_bridge");
    ros::NodeHandle nh;
    std::string name;
    if(!nh.getParam("/robot_name", name)) {
        ROS_ERROR("GPS bridge node: Failed to get param 'name'");
        if(!nh.getParam("/localization_bridge/name", name)) {
            ROS_ERROR("GPS bridge node: Failed to get param 'localization_bridge/name'");
            return 1;
        }
    }
    LocalizationBridge bridge(nh, name);
    ros::Rate loop_rate(60);
    while (ros::ok()) {
        ros::spinOnce();
        loop_rate.sleep();
    }

    return 0;
}