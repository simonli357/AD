#include "Client.hpp"
#include <cstdint>
#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <thread>
#include <vector>

Client::Client(const char *server_ip, const uint16_t server_port) : ip(server_ip), port(server_port) {
    client_socket = socket(AF_INET, SOCK_STREAM, 0);
    address.sin_family = AF_INET;
    address.sin_port = htons(server_port);
    inet_pton(AF_INET, server_ip, &address.sin_addr);
    data_types.push_back(0x01); // String
    data_actions[data_types[0]] = &Client::parse_string;
}

Client::~Client() {
    alive = false;
    if (client_socket != -1) {
        close(client_socket);
    }
    if (receive.joinable()) {
        receive.join();
    }
}

void Client::initialize() {
    if(connect(client_socket, (struct sockaddr*)& address, sizeof(address)) == -1) {}
    receive = std::thread(&Client::listen, this);
}

void Client::listen() {
    const size_t buffer_size = 1024;
    std::vector<uint8_t> buffer(buffer_size);
    while (alive) {
        uint8_t type;
        uint32_t length = 0;
        // Receive the header: length (4 bytes) + type (1 byte)
        ssize_t bytes_received = 0;
        while (bytes_received < 5) {
            bytes_received = recv(client_socket, buffer.data() + bytes_received, 5 - bytes_received, 0);
            if (bytes_received <= 0) {
                continue;
            }
        }
        std::memcpy(&length, buffer.data(), 4);
        type = buffer[4];
        length = ntohl(length);
        // Receive the actual data based on the length from header
        std::vector<uint8_t> data(length);
        size_t total_bytes_received = 0;
        while (total_bytes_received < length) {
            ssize_t bytes = recv(client_socket, data.data() + total_bytes_received, length - total_bytes_received, 0);
            if (bytes <= 0) {
                break;
            }
            total_bytes_received += bytes;
        }
        if (total_bytes_received == length) {
            // Process data if full message is received
            auto it = data_actions.find(type);
            if (it != data_actions.end()) {
                it->second(this, data);
            }
        } 
    }
}

void Client::send_string(const std::string& str) {
    uint32_t length = str.size();
    size_t total_size = 4 + 1 + length;
    uint32_t big_endian_length = htonl(length);
    std::vector<uint8_t> full_message(total_size);
    std::memcpy(full_message.data(), &big_endian_length, 4);
    full_message[4] = data_types[0];
    std::memcpy(full_message.data() + 5, str.data(), length);
    send(client_socket, full_message.data(), full_message.size(), 0);
}

void Client::parse_string(std::vector<uint8_t>& data) {
    std::string decoded_string(data.begin(), data.end());
    std::cout << decoded_string << std::endl;
}

/* void Client::sendImage(const sensor_msgs::Image& img) { */
/*     std::vector<uint8_t> buffer = serializeImage(img); */
/*     if(send(client_socket, buffer.data(), buffer.size(), 0) == -1) { */
/*         ROS_ERROR("Error sending image"); */
/*     } */
/* } */

/* std::vector<uint8_t> Client::serializeImage(const sensor_msgs::Image& img) { */
/*     uint32_t size = ros::serialization::serializationLength(img); */
/*     std::vector<uint8_t> buffer(size); */
/*     ros::serialization::OStream stream(buffer.data(), size); */
/*     ros::serialization::serialize(stream, img); */
/*     return buffer; */
/* } */
