#include "TcpClient.hpp"
#include "ros/serialization.h"
#include "sensor_msgs/Image.h"
#include "service_calls/GoToCmdSrvResponse.hpp"
#include "service_calls/GoToSrvResponse.hpp"
#include "service_calls/SrvRequest.hpp"
#include "service_calls/WaypointsSrvResponse.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <netinet/in.h>
#include <sys/socket.h>
#include <thread>
#include <vector>

TcpClient::TcpClient(const char *server_ip, const uint16_t server_port, const size_t buffer_size) : buffer_size(buffer_size) {
	client_socket = socket(AF_INET, SOCK_STREAM, 0);
	address.sin_family = AF_INET;
	address.sin_port = htons(server_port);
	inet_pton(AF_INET, server_ip, &address.sin_addr);
	set_data_types();
	set_data_actions();
	receive = std::thread(&TcpClient::initialize, this);
}

TcpClient::TcpClient(const size_t buffer_size, const char *client_type) : buffer_size(buffer_size), client_type(client_type) {
	client_socket = socket(AF_INET, SOCK_STREAM, 0);
	address.sin_family = AF_INET;
	address.sin_port = htons(49153); // Default port
	set_data_types();
	set_data_actions();
	inet_pton(AF_INET, "127.0.0.1", &address.sin_addr); // Default address
	receive = std::thread(&TcpClient::initialize, this);
}

TcpClient::~TcpClient() {
	alive = false;
	if (client_socket != -1) {
		close(client_socket);
	}
	if (receive.joinable()) {
		receive.join();
	}
}

void TcpClient::set_data_types() {
	data_types.push_back(0x01); // std::string
	data_types.push_back(0x02); // Image rgb
	data_types.push_back(0x03); // Image depth
	data_types.push_back(0x04); // Road Objects
	data_types.push_back(0x05); // Waypoints
	data_types.push_back(0x06); // Signs
	data_types.push_back(0x07); // Messages
	data_types.push_back(0x08); // GoToSrv
	data_types.push_back(0x09); // GoToCmdSrv
	data_types.push_back(0x0a); // SetStatesSrv
	data_types.push_back(0x0b); // WaypointsSrv
    data_types.push_back(0x0c); // StartSrv
}

void TcpClient::set_data_actions() {
	data_actions[data_types[0]] = &TcpClient::parse_string;			// std::string
	data_actions[data_types[7]] = &TcpClient::parse_go_to_srv;		// GoToSrv
	data_actions[data_types[8]] = &TcpClient::parse_go_to_cmd_srv;	// GoToCmdSrc
	data_actions[data_types[9]] = &TcpClient::parse_set_states_srv; // SetStatesSrv
	data_actions[data_types[10]] = &TcpClient::parse_waypoints_srv; // SetStatesSrv
    data_actions[data_types[11]] = &TcpClient::parse_start_srv;     // StartSrv
}

void TcpClient::initialize() {
	std::cout << "Connecting to GUI \n" << std::endl;
	while (true) {
		if (connect(client_socket, (struct sockaddr *)&address, sizeof(address)) != -1) {
			break;
		}
	}
	std::cout << "Connection request established with GUI\n" << std::endl;
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
	if (client_type != nullptr) {
		send_type(client_type);
	}
	listen();
}

void TcpClient::listen() {
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

// ------------------- //
// Data Storage
// ------------------- //

std::queue<std::string> &TcpClient::get_strings() { return strings; }
std::queue<SrvRequest::GoToSrv> &TcpClient::get_go_to_srv_msgs() { return go_to_srv_msgs; }
std::queue<SrvRequest::GoToCmdSrv> &TcpClient::get_go_to_cmd_srv_msgs() { return go_to_cmd_srv_msgs; }
std::queue<SrvRequest::SetStatesSrv> &TcpClient::get_set_states_srv_msgs() { return set_states_srv_msgs; }
std::queue<SrvRequest::WaypointsSrv> &TcpClient::get_waypoints_srv_msgs() { return waypoints_srv_msgs; }
std::queue<bool> &TcpClient::get_start_srv_msgs() { return start_srv_msgs; }

// ------------------- //
// Encoding
// ------------------- //

void TcpClient::send_type(const std::string &str) {
	uint32_t length = str.size();
	size_t total_size = header_size + length;
	std::vector<uint8_t> full_message(total_size);
	std::memcpy(full_message.data(), &length, message_size);
	full_message[4] = data_types[0];
	std::memcpy(full_message.data() + header_size, str.data(), length);
	send(client_socket, full_message.data(), full_message.size(), 0);
}

void TcpClient::send_string(const std::string &str) {
	if (canSend) {
		uint32_t length = str.size();
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[0];
		std::memcpy(full_message.data() + header_size, str.data(), length);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_image_rgb(const sensor_msgs::Image &img) {
	if (canSend) {
		uint32_t length = ros::serialization::serializationLength(img);
		std::vector<uint8_t> image(length);
		ros::serialization::OStream stream(image.data(), length);
		ros::serialization::serialize(stream, img);
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[1];
		std::memcpy(full_message.data() + header_size, image.data(), length);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_image_depth(const sensor_msgs::Image &img) {
	if (canSend) {
		uint32_t length = ros::serialization::serializationLength(img);
		std::vector<uint8_t> image(length);
		ros::serialization::OStream stream(image.data(), length);
		ros::serialization::serialize(stream, img);
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[2];
		std::memcpy(full_message.data() + header_size, image.data(), length);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_road_object(const std_msgs::Float32MultiArray &array) {
	if (canSend) {
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[3];
		std::memcpy(full_message.data() + header_size, arr.data(), length);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_waypoint(const std_msgs::Float32MultiArray &array) {
	if (canSend) {
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[4];
		std::memcpy(full_message.data() + header_size, arr.data(), length);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_sign(const std_msgs::Float32MultiArray &array) {
	if (canSend) {
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[5];
		std::memcpy(full_message.data() + header_size, arr.data(), length);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_message(const std_msgs::String &msg) {
	if (canSend) {
		uint32_t length = ros::serialization::serializationLength(msg);
		std::vector<uint8_t> message(length);
		ros::serialization::OStream stream(message.data(), length);
		ros::serialization::serialize(stream, msg);
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[6];
		std::memcpy(full_message.data() + header_size, message.data(), length);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_go_to_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals) {
	if (canSend) {
		std::vector<uint8_t> bytes = GoToSrvResponse(data_types[7], state_refs, input_refs, wp_attributes, wp_normals).serialize();
		send(client_socket, bytes.data(), bytes.size(), 0);
	}
}

void TcpClient::send_go_to_cmd_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals, bool success) {
	if (canSend) {
		std::vector<uint8_t> bytes = GoToCmdSrvResponse(data_types[8], state_refs, input_refs, wp_attributes, wp_normals, success).serialize();
		send(client_socket, bytes.data(), bytes.size(), 0);
	}
}

void TcpClient::send_set_states_srv(bool success) {
	if (canSend) {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[9];
        full_message[5] = static_cast<uint8_t>(success);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_waypoints_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals) {
	if (canSend) {
		std::vector<uint8_t> bytes = WaypointsSrvResponse(data_types[10], state_refs, input_refs, wp_attributes, wp_normals).serialize();
		send(client_socket, bytes.data(), bytes.size(), 0);
	}
}

void TcpClient::send_start_srv(bool started) {
	if (canSend) {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = data_types[11];
        full_message[5] = static_cast<uint8_t>(started);
		send(client_socket, full_message.data(), full_message.size(), 0);
	}
}

// ------------------- //
// Decoding
// ------------------- //

void TcpClient::parse_string(std::vector<uint8_t> &bytes) {
	std::string decoded_string(bytes.begin(), bytes.end());
	if (decoded_string == "ack") {
		canSend = true;
		std::cout << client_type << " successfully connected to GUI.\n" << std::endl;
		return;
	}
	strings.push(decoded_string);
}

void TcpClient::parse_go_to_srv(std::vector<uint8_t> &bytes) {
    go_to_srv_msgs.push(SrvRequest(bytes).parse_go_to_srv());
}

void TcpClient::parse_go_to_cmd_srv(std::vector<uint8_t> &bytes) {
    go_to_cmd_srv_msgs.push(SrvRequest(bytes).parse_go_to_cmd_srv());
}

void TcpClient::parse_set_states_srv(std::vector<uint8_t> &bytes) {
    set_states_srv_msgs.push(SrvRequest(bytes).parse_set_states_srv());
}

void TcpClient::parse_waypoints_srv(std::vector<uint8_t> &bytes) {
    waypoints_srv_msgs.push(SrvRequest(bytes).parse_waypoints_srv());
}

void TcpClient::parse_start_srv(std::vector<uint8_t> &bytes) {
    start_srv_msgs.push(true);
}
