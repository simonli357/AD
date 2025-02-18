#include "TcpClient.hpp"
#include <ros/ros.h>
#include "msg/ParamsMsg.hpp"
#include "msg/Lane2Msg.hpp"
#include "msg/TriggerMsg.hpp"
#include "ros/serialization.h"
#include "sensor_msgs/Image.h"
#include "service_calls/GoToCmdSrv.hpp"
#include "service_calls/GoToSrv.hpp"
#include "service_calls/SetStatesSrv.hpp"
#include "service_calls/WaypointsSrv.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "utils/Lane2.h"
#include <arpa/inet.h>
#include <cstdint>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <netinet/in.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <sys/socket.h>
#include <thread>
#include <vector>

TcpClient::TcpClient(const size_t buffer_size, const std::string client_type, const std::string ip_address)
	: buffer_size(buffer_size), client_type(client_type), server_address(ip_address) {
	create_udp_socket();
	set_tcp_data_types();
	set_udp_data_types();
	set_tcp_data_actions();
	receive = std::thread(&TcpClient::initialize, this);
	poll = std::thread(&TcpClient::poll_connection, this);
}

TcpClient::~TcpClient() {
	alive = false;
	connected = false;
	if (tcp_socket != -1) {
		close(tcp_socket);
	}
	if (receive.joinable()) {
		receive.join();
	}
	if (poll.joinable()) {
		poll.join();
	}
}

// ------------------- //
// Utility Methods
// ------------------- //

void TcpClient::create_tcp_socket() {
	tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	tcp_address.sin_family = AF_INET;
	tcp_address.sin_port = htons(49153);
	inet_pton(AF_INET, server_address.c_str(), &tcp_address.sin_addr);
}

void TcpClient::create_udp_socket() {
	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	udp_address.sin_family = AF_INET;
	udp_address.sin_port = htons(49154);
	inet_pton(AF_INET, server_address.c_str(), &udp_address.sin_addr);
}

void TcpClient::set_tcp_data_types() {
	tcp_data_types.push_back(0x01); // std::string
	tcp_data_types.push_back(0x02); // Trigger
	tcp_data_types.push_back(0x03); // Messages
	tcp_data_types.push_back(0x04); // GoTo Srv
	tcp_data_types.push_back(0x05); // GoToCmd Srv
	tcp_data_types.push_back(0x06); // SetStates Srv
	tcp_data_types.push_back(0x07); // Waypoints Srv
	tcp_data_types.push_back(0x08); // Start Srv
    tcp_data_types.push_back(0x09); // Params
}

void TcpClient::set_udp_data_types() {
	udp_data_types.push_back(0x01); // Lane2
	udp_data_types.push_back(0x02); // Road Objects
	udp_data_types.push_back(0x03); // Waypoints
	udp_data_types.push_back(0x04); // Signs
	udp_data_types.push_back(0x05); // RGB Images
	udp_data_types.push_back(0x06); // Depth Images
}

void TcpClient::set_tcp_data_actions() {
	tcp_data_actions[tcp_data_types[0]] = &TcpClient::parse_string;			// std::string
	tcp_data_actions[tcp_data_types[1]] = &TcpClient::parse_trigger_msg;	// Trigger
	tcp_data_actions[tcp_data_types[3]] = &TcpClient::parse_go_to_srv;		// GoToSrv
	tcp_data_actions[tcp_data_types[4]] = &TcpClient::parse_go_to_cmd_srv;	// GoToCmdSrc
	tcp_data_actions[tcp_data_types[5]] = &TcpClient::parse_set_states_srv; // SetStatesSrv
	tcp_data_actions[tcp_data_types[6]] = &TcpClient::parse_waypoints_srv;	// SetStatesSrv
	tcp_data_actions[tcp_data_types[7]] = &TcpClient::parse_start_srv;		// StartSrv
}

void TcpClient::initialize() {
	create_tcp_socket();
	std::cout << "Connecting to GUI \n" << std::endl;
	while (true) {
		if (connect(tcp_socket, (struct sockaddr *)&tcp_address, sizeof(tcp_address)) != -1) {
			break;
		}
	}
	std::cout << "Connection request established with GUI\n" << std::endl;
	connected = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(250));
	if (!client_type.empty()) {
		send_type(client_type);
	}
	listen();
}

void TcpClient::poll_connection() {
	while (alive) {
		char buffer[32];
		if (connected && recv(tcp_socket, buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT) == 0) {
			std::cout << "GUI disconnected\n" << std::endl;
			connected = false;
			pthread_cancel(receive.native_handle());
			if (receive.joinable()) {
				receive.join();
			}
			close(tcp_socket);
			receive = std::thread(&TcpClient::initialize, this);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
}

void TcpClient::listen() {
	std::vector<uint8_t> buffer(buffer_size);
	while (connected) {
		uint8_t type;
		uint32_t length = 0;
		// Receive the header
		ssize_t bytes_received = 0;
		while (bytes_received < header_size) {
			bytes_received = recv(tcp_socket, buffer.data() + bytes_received, 5 - bytes_received, 0);
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
			ssize_t bytes = recv(tcp_socket, data.data() + total_bytes_received, length - total_bytes_received, 0);
			if (bytes <= 0) {
				break;
			}
			total_bytes_received += bytes;
		}
		// Process data if full message is received
		if (total_bytes_received == length) {
			auto it = tcp_data_actions.find(type);
			if (it != tcp_data_actions.end()) {
				it->second(this, data);
			}
		}
	}
}

// ------------------- //
// Data Storage
// ------------------- //

std::queue<std::string> &TcpClient::get_strings() { return strings; }
std::queue<std::unique_ptr<TriggerMsg>> &TcpClient::get_trigger_msgs() { return trigger_msgs; }
std::queue<std::unique_ptr<ParamsMsg>> &TcpClient::get_params_msgs() { return params_msgs; }
std::queue<std::unique_ptr<GoToSrv>> &TcpClient::get_go_to_srv_msgs() { return go_to_srv_msgs; }
std::queue<std::unique_ptr<GoToCmdSrv>> &TcpClient::get_go_to_cmd_srv_msgs() { return go_to_cmd_srv_msgs; }
std::queue<std::unique_ptr<SetStatesSrv>> &TcpClient::get_set_states_srv_msgs() { return set_states_srv_msgs; }
std::queue<std::unique_ptr<WaypointsSrv>> &TcpClient::get_waypoints_srv_msgs() { return waypoints_srv_msgs; }
std::queue<bool> &TcpClient::get_start_srv_msgs() { return start_srv_msgs; }

// ------------------- //
// TCP Encoding
// ------------------- //

void TcpClient::send_type(std::string &str) {
	uint32_t length = str.size();
	size_t total_size = header_size + length;
	std::vector<uint8_t> full_message(total_size);
	std::memcpy(full_message.data(), &length, message_size);
	full_message[4] = tcp_data_types[0];
	std::memcpy(full_message.data() + header_size, str.data(), length);
	send(tcp_socket, full_message.data(), full_message.size(), 0);
}

void TcpClient::send_string(std::string &str) {
	if (tcp_can_send) {
		uint32_t length = str.size();
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[0];
		std::memcpy(full_message.data() + header_size, str.data(), length);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_trigger(std_srvs::Trigger &trigger) {
	if (tcp_can_send) {
		std::vector<uint8_t> bytes = TriggerMsg(trigger).serialize(tcp_data_types[1]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	}
}

void TcpClient::send_message(const std_msgs::String &msg) {
	if (tcp_can_send) {
		uint32_t length = ros::serialization::serializationLength(msg);
		std::vector<uint8_t> message(length);
		ros::serialization::OStream stream(message.data(), length);
		ros::serialization::serialize(stream, msg);
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[2];
		std::memcpy(full_message.data() + header_size, message.data(), length);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_go_to_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals) {
	if (tcp_can_send) {
		std::vector<uint8_t> bytes = GoToSrv(state_refs, input_refs, wp_attributes, wp_normals).serialize(tcp_data_types[3]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	}
}

void TcpClient::send_go_to_cmd_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals, bool success) {
	if (tcp_can_send) {
		std::vector<uint8_t> bytes = GoToCmdSrv(state_refs, input_refs, wp_attributes, wp_normals, success).serialize(tcp_data_types[4]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	}
}

void TcpClient::send_set_states_srv(bool success) {
	if (tcp_can_send) {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[5];
		full_message[5] = static_cast<uint8_t>(success);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_waypoints_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals) {
	if (tcp_can_send) {
		std::vector<uint8_t> bytes = WaypointsSrv(state_refs, input_refs, wp_attributes, wp_normals).serialize(tcp_data_types[6]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	}
}

void TcpClient::send_start_srv(bool started) {
	if (tcp_can_send) {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[7];
		full_message[5] = static_cast<uint8_t>(started);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	}
}

void TcpClient::send_params(std::vector<double> &state_refs, std::vector<double> &attributes) {
    if (tcp_can_send) {
        std::vector<uint8_t> bytes = ParamsMsg(state_refs, attributes).serialize(tcp_data_types[8]);
        send(tcp_socket, bytes.data(), bytes.size(), 0);
    }
}

// ------------------- //
// UDP Encoding
// ------------------- //

void TcpClient::send_lane2(const utils::Lane2 &lane) {
	std_msgs::Header header = lane.header;
	float center = lane.center;
	int stopline = lane.stopline;
	bool crosswalk = lane.crosswalk;
	std::vector<uint8_t> bytes = Lane2Msg(header, center, stopline, crosswalk, false).serialize(udp_data_types[0]);

	std::vector<uint8_t> segment(MAX_DGRAM, 0);
    std::memcpy(segment.data(), bytes.data(), bytes.size());

    sendto(udp_socket, segment.data(), segment.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
}

void TcpClient::send_road_object(const std_msgs::Float32MultiArray &array) {
	uint32_t length = ros::serialization::serializationLength(array);
	std::vector<uint8_t> arr(length);
	ros::serialization::OStream stream(arr.data(), length);
	ros::serialization::serialize(stream, array);

	std::vector<uint8_t> bytes(MAX_DGRAM, 0);
	std::memcpy(bytes.data(), &length, message_size);
	bytes[4] = udp_data_types[1];
	std::memcpy(bytes.data() + header_size, arr.data(), length);

	sendto(udp_socket, bytes.data(), bytes.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
}

void TcpClient::send_waypoint(const std_msgs::Float32MultiArray &array) {
	uint32_t length = ros::serialization::serializationLength(array);
	std::vector<uint8_t> arr(length);
	ros::serialization::OStream stream(arr.data(), length);
	ros::serialization::serialize(stream, array);

	std::vector<uint8_t> bytes(MAX_DGRAM, 0);
	std::memcpy(bytes.data(), &length, message_size);
	bytes[4] = udp_data_types[2];
	std::memcpy(bytes.data() + header_size, arr.data(), length);

	sendto(udp_socket, bytes.data(), bytes.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
}

void TcpClient::send_sign(const std_msgs::Float32MultiArray &array) {
	uint32_t length = ros::serialization::serializationLength(array);
	std::vector<uint8_t> arr(length);
	ros::serialization::OStream stream(arr.data(), length);
	ros::serialization::serialize(stream, array);
	
	std::vector<uint8_t> bytes(MAX_DGRAM, 0);
	std::memcpy(bytes.data(), &length, message_size);
	bytes[4] = tcp_data_types[3];
	std::memcpy(bytes.data() + header_size, arr.data(), length);
	
	sendto(udp_socket, bytes.data(), bytes.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
}

void TcpClient::send_image_rgb(const sensor_msgs::Image &img) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::BGR8);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("send_image_rgb(): cv_bridge exception: %s", e.what());
		return;
	}
	std::vector<uchar> image;
	cv::imencode(".jpg", cv_ptr->image, image, {cv::IMWRITE_JPEG_QUALITY, 70});
	uint32_t length = image.size();
	uint8_t total_segments = std::ceil(static_cast<float>(length + header_size) / MAX_DGRAM);
	if (total_segments == 1) {
		std::vector<uint8_t> segment(MAX_DGRAM, 0);
        std::memcpy(segment.data(), &length, message_size);
        segment[4] = udp_data_types[4];
		std::memcpy(segment.data() + header_size, &image[0], image.size());
		sendto(udp_socket, segment.data(), segment.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	}
}

void TcpClient::send_image_depth(const sensor_msgs::Image &img) {
	cv_bridge::CvImagePtr cv_ptr;
	try {
		cv_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::TYPE_16UC1);
	} catch (cv_bridge::Exception &e) {
		ROS_ERROR("send_image_depth(): cv_bridge exception: %s", e.what());
		return;
	}
	std::vector<uchar> image;
	cv::imencode(".png", cv_ptr->image, image, {cv::IMWRITE_PNG_COMPRESSION, 4});
	uint32_t length = image.size();
	uint8_t total_segments = std::ceil(static_cast<float>(length + header_size) / MAX_DGRAM);
	if (total_segments == 1) {
		std::vector<uint8_t> segment(MAX_DGRAM, 0);
        std::memcpy(segment.data(), &length, message_size);
        segment[4] = udp_data_types[5];
		std::memcpy(segment.data() + header_size, &image[0], image.size());
		sendto(udp_socket, segment.data(), segment.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	}
}

// ------------------- //
// TCP Decoding
// ------------------- //

void TcpClient::parse_string(std::vector<uint8_t> &bytes) {
	std::string decoded_string(bytes.begin(), bytes.end());
	if (decoded_string == "ack") {
		tcp_can_send = true;
		std::cout << client_type << " successfully connected to GUI.\n" << std::endl;
		return;
	}
	strings.push(decoded_string);
}

void TcpClient::parse_trigger_msg(std::vector<uint8_t> &bytes) { trigger_msgs.push(TriggerMsg().deserialize(bytes)); }

void TcpClient::parse_params_msg(std::vector<uint8_t> &bytes) { params_msgs.push(ParamsMsg().deserialize(bytes)); }

void TcpClient::parse_go_to_srv(std::vector<uint8_t> &bytes) { go_to_srv_msgs.push(GoToSrv().deserialize(bytes)); }

void TcpClient::parse_go_to_cmd_srv(std::vector<uint8_t> &bytes) { go_to_cmd_srv_msgs.push(GoToCmdSrv().deserialize(bytes)); }

void TcpClient::parse_set_states_srv(std::vector<uint8_t> &bytes) { set_states_srv_msgs.push(SetStatesSrv().deserialize(bytes)); }

void TcpClient::parse_waypoints_srv(std::vector<uint8_t> &bytes) { waypoints_srv_msgs.push(WaypointsSrv().deserialize(bytes)); }

void TcpClient::parse_start_srv(std::vector<uint8_t> &bytes) {
	std::string decoded_string(bytes.begin(), bytes.end());
	if (decoded_string == "start") {
		start_srv_msgs.push(true);
	} else if (decoded_string == "stop") {
		start_srv_msgs.push(false);
	}
}
