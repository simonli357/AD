#include "Client.hpp"
#include "ros/serialization.h"
#include "sensor_msgs/Image.h"
#include "std_msgs/String.h"
#include "std_msgs/Float32MultiArray.h"
#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <vector>

Client::Client(const char *server_ip, const uint16_t server_port, const size_t buffer_size) : buffer_size(buffer_size) {
	client_socket = socket(AF_INET, SOCK_STREAM, 0);
	address.sin_family = AF_INET;
	address.sin_port = htons(server_port);
	inet_pton(AF_INET, server_ip, &address.sin_addr);
	data_types.push_back(0x01); // std::string
	data_types.push_back(0x02); // Image
	data_types.push_back(0x03); // Float32MultiArray
    data_types.push_back(0x04); // String
	data_types.push_back(0x05); // Lane2
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
	if (connect(client_socket, (struct sockaddr *)&address, sizeof(address)) == -1) {
	}
	receive = std::thread(&Client::listen, this);
}

void Client::listen() {
	std::vector<uint8_t> buffer(buffer_size);
	while (alive) {
		uint8_t type;
		uint32_t length = 0;
		// Receive the header
		ssize_t bytes_received = 0;
		while (bytes_received < header_size) {
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
		// Process data if full message is received
		if (total_bytes_received == length) {
			auto it = data_actions.find(type);
			if (it != data_actions.end()) {
				it->second(this, data);
			}
		}
	}
}

std::queue<std::string> &Client::get_strings() { return strings; }

void Client::send_string(const std::string &str) {
	uint32_t length = str.size();
	uint32_t big_endian_length = htonl(length);
	size_t total_size = header_size + length;
	std::vector<uint8_t> full_message(total_size);
	std::memcpy(full_message.data(), &big_endian_length, message_size);
	full_message[4] = data_types[0];
	std::memcpy(full_message.data() + header_size, str.data(), length);
	send(client_socket, full_message.data(), full_message.size(), 0);
}

void Client::send_image(const sensor_msgs::Image &img) {
	uint32_t length = ros::serialization::serializationLength(img);
	std::vector<uint8_t> image(length);
	ros::serialization::OStream stream(image.data(), length);
	ros::serialization::serialize(stream, img);
	uint32_t big_endian_length = htonl(length);
	size_t total_size = header_size + length;
	std::vector<uint8_t> full_message(total_size);
	std::memcpy(full_message.data(), &big_endian_length, message_size);
	full_message[4] = data_types[1];
	std::memcpy(full_message.data() + header_size, image.data(), length);
	send(client_socket, full_message.data(), full_message.size(), 0);
    std::this_thread::sleep_for(std::chrono::milliseconds(30));
}

void Client::send_float32_multi_array(const std_msgs::Float32MultiArray &array) {
    uint32_t length = ros::serialization::serializationLength(array);
    std::vector<uint8_t> arr(length);
    ros::serialization::OStream stream(arr.data(), length);
    ros::serialization::serialize(stream, array);
    uint32_t big_endian_length = htonl(length);
    size_t total_size = header_size + length;
    std::vector<uint8_t> full_message(total_size);
    std::memcpy(full_message.data(), &big_endian_length, message_size);
    full_message[4] = data_types[2];
    std::memcpy(full_message.data() + header_size, arr.data(), length);
    send(client_socket, full_message.data(), full_message.size(), 0);
}

void Client::send_message(const std_msgs::String &msg) {
    uint32_t length = ros::serialization::serializationLength(msg);
    std::vector<uint8_t> message(length);
    ros::serialization::OStream stream(message.data(), length);
    ros::serialization::serialize(stream, msg);
    uint32_t big_endian_length = htonl(length);
    size_t total_size = header_size + length;
    std::vector<uint8_t> full_message(total_size);
    std::memcpy(full_message.data(), &big_endian_length, message_size);
    full_message[4] = data_types[3];
    std::memcpy(full_message.data() + header_size, message.data(), length);
    send(client_socket, full_message.data(), full_message.size(), 0);
}

void Client::parse_string(std::vector<uint8_t> &data) {
	std::string decoded_string(data.begin(), data.end());
	strings.push(decoded_string);
}
