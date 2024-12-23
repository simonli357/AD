#include "Client.hpp"
#include "ros/console.h"
#include "ros/ros.h"
#include "ros/serialization.h"
#include "sensor_msgs/Image.h"
#include <cstdint>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <thread>
#include <vector>

Client::Client(const char *receiver_ip, const uint16_t receiver_port) : ip(receiver_ip), port(receiver_port) {
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    address.sin_family = AF_INET;
    address.sin_port = htons(receiver_port);
}

void Client::initialize() {
    while(connect(client_socket, (struct sockaddr*)& address, sizeof(address)) == -1) {
        ROS_INFO("Connecting to server");
    }
    receive = std::thread(&Client::listen, this);
}

void Client::listen() {
       
}

void Client::sendImage(const sensor_msgs::Image& img) {
    std::vector<uint8_t> buffer = serializeImage(img);
    if(send(client_socket, buffer.data(), buffer.size(), 0) == -1) {
        ROS_ERROR("Error sending image");
    }
}

std::vector<uint8_t> Client::serializeImage(const sensor_msgs::Image& img) {
    uint32_t size = ros::serialization::serializationLength(img);
    std::vector<uint8_t> buffer(size);
    ros::serialization::OStream stream(buffer.data(), size);
    ros::serialization::serialize(stream, img);
    return buffer;
}
