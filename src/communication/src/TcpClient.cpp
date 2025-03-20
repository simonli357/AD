#include "TcpClient.hpp"
#include "msg/Lane2Msg.hpp"
#include "msg/ParamsMsg.hpp"
#include "msg/RunMsg.hpp"
#include "msg/SWLoadMsg.hpp"
#include "msg/TriggerMsg.hpp"
#include "ros/serialization.h"
#include "service_calls/GoToCmdSrv.hpp"
#include "service_calls/GoToSrv.hpp"
#include "service_calls/SetStatesSrv.hpp"
#include "service_calls/WaypointsSrv.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64MultiArray.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "utils/Lane2.h"
#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <hwloc.h>
#include <iostream>
#include <netinet/in.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sys/socket.h>
#include <thread>
#include <vector>

TcpClient::TcpClient(bool use_tcp, const std::string client_type, const std::string ip_address) : client_type(client_type), server_address(ip_address) {
	create_udp_socket();
	set_udp_data_types();
	if (use_tcp) {
		set_tcp_data_types();
		set_tcp_data_actions();
		receiver = std::thread(&TcpClient::initialize, this);
		sender = std::thread(&TcpClient::send_data, this);
		perfmon = std::thread(&TcpClient::send_swload, this);
	}
}

TcpClient::~TcpClient() {
	alive = false;
	connected = false;
	if (tcp_socket != -1) {
		close(tcp_socket);
	}
	if (receiver.joinable()) {
		receiver.join();
	}
	if (sender.joinable()) {
		sender.join();
	}
	if (perfmon.joinable()) {
		perfmon.join();
	}
}

// ------------------- //
// Utility Methods
// ------------------- //

void TcpClient::create_tcp_socket() {
	tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	tcp_address.sin_family = AF_INET;
	tcp_address.sin_port = htons(tcp_port);
	inet_pton(AF_INET, server_address.c_str(), &tcp_address.sin_addr);
	int flags = fcntl(tcp_socket, F_GETFL, 0);
	fcntl(tcp_socket, F_SETFL, flags | O_NONBLOCK);
}

void TcpClient::create_udp_socket() {
	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	udp_address.sin_family = AF_INET;
	udp_address.sin_port = htons(udp_port);
	inet_pton(AF_INET, server_address.c_str(), &udp_address.sin_addr);
	int flags = fcntl(udp_socket, F_GETFL, 0);
	fcntl(udp_socket, F_SETFL, flags | O_NONBLOCK);
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
	tcp_data_types.push_back(0x0a); // Run params
}

void TcpClient::set_udp_data_types() {
	udp_data_types.push_back(0x01); // Lane2
	udp_data_types.push_back(0x02); // Road Objects
	udp_data_types.push_back(0x03); // Waypoints
	udp_data_types.push_back(0x04); // Signs
	udp_data_types.push_back(0x05); // RGB Images
	udp_data_types.push_back(0x06); // Depth Images
	udp_data_types.push_back(0x07); // Steer
	udp_data_types.push_back(0x08); // SWLoad
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
	while (alive) {
		create_tcp_socket();
		std::cout << "Connecting to GUI \n" << std::endl;
		while (true) {
			if (connect(tcp_socket, (struct sockaddr *)&tcp_address, sizeof(tcp_address)) != -1) {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		std::cout << "Connection request established with GUI \n" << std::endl;
		connected = true;
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
		if (!client_type.empty()) {
			send_type(client_type);
		}
		listen();
	}
}

void TcpClient::listen() {
	std::vector<uint8_t> header_buffer(5); // Fixed size for header (4 + 1 bytes)
	while (connected) {
		// --- Header Reception ---
		ssize_t total_header_received = 0;
		while (total_header_received < 5) {
			ssize_t bytes = recv(tcp_socket, header_buffer.data() + total_header_received, 5 - total_header_received, 0);

			if (bytes > 0) {
				total_header_received += bytes;
			} else if (bytes == 0) {
				// Connection closed
				connected = false;
				break;
			} else { // bytes == -1
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					// Non-blocking retry
					usleep(10000); // 10ms delay (adjust as needed)
					continue;
				} else {
					// Handle other errors
					connected = false;
					break;
				}
			}
		}

		if (!connected || total_header_received != 5)
			break;

		// --- Process Header ---
		uint32_t length;
		uint8_t type;
		std::memcpy(&length, header_buffer.data(), 4);
		type = header_buffer[4];

		// --- Data Reception ---
		std::vector<uint8_t> data_buffer(length);
		ssize_t total_data_received = 0;
		while (total_data_received < length) {
			ssize_t bytes = recv(tcp_socket, data_buffer.data() + total_data_received, length - total_data_received, 0);

			if (bytes > 0) {
				total_data_received += bytes;
			} else if (bytes == 0) {
				connected = false;
				break;
			} else {
				if (errno == EAGAIN || errno == EWOULDBLOCK) {
					usleep(10000);
					continue;
				} else {
					connected = false;
					break;
				}
			}
		}

		if (total_data_received == length) {
			auto handler = tcp_data_actions.find(type);
			if (handler != tcp_data_actions.end()) {
				handler->second(this, data_buffer);
			}
		} else {
			connected = false;
		}
	}
    run_sent = false;
	tcp_can_send = false;
}

void TcpClient::send_data() {
	while (alive) {
		if (!stream_tasks.empty() && tcp_can_send) {
			std::any stream_task;
			if (stream_tasks.try_pop(stream_task)) {
				std::function<void()> task = std::any_cast<std::function<void()>>(stream_task);
				task();
			}
			continue;
		}
		if (!dgram_tasks.empty()) {
			std::any dgram_task;
			if (dgram_tasks.try_pop(dgram_task)) {
				std::function<void()> task = std::any_cast<std::function<void()>>(dgram_task);
				task();
			}
			continue;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(16));
	}
}

template <typename Callable> void TcpClient::add_stream_task(Callable &&lambda) { stream_tasks.push(std::function<void()>(std::forward<Callable>(lambda))); }

template <typename Callable> void TcpClient::add_dgram_task(Callable &&lambda) { dgram_tasks.push(std::function<void()>(std::forward<Callable>(lambda))); }

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

void TcpClient::send_type(const std::string &str) {
	uint32_t length = str.size();
	size_t total_size = header_size + length;
	std::vector<uint8_t> full_message(total_size);
	std::memcpy(full_message.data(), &length, message_size);
	full_message[4] = tcp_data_types[0];
	std::memcpy(full_message.data() + header_size, str.data(), length);
	send(tcp_socket, full_message.data(), full_message.size(), 0);
}

void TcpClient::send_string(const std::string &str) {
	auto fn = [this, str]() {
		uint32_t length = str.size();
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[0];
		std::memcpy(full_message.data() + header_size, str.data(), length);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_trigger(const std_srvs::Trigger &trigger) {
	auto fn = [this, trigger]() {
		std::vector<uint8_t> bytes = TriggerMsg(trigger).serialize(tcp_data_types[1]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_message(const std_msgs::String &msg) {
	auto fn = [this, msg]() {
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
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_go_to_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals) {
	auto fn = [this, state_refs, input_refs, wp_attributes, wp_normals]() {
		std::vector<uint8_t> bytes = GoToSrv(state_refs, input_refs, wp_attributes, wp_normals).serialize(tcp_data_types[3]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_go_to_cmd_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals, bool success) {
	auto fn = [this, state_refs, input_refs, wp_attributes, wp_normals, success]() {
		std::vector<uint8_t> bytes = GoToCmdSrv(state_refs, input_refs, wp_attributes, wp_normals, success).serialize(tcp_data_types[4]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_set_states_srv(bool success) {
	auto fn = [this, success]() {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[5];
		full_message[5] = static_cast<uint8_t>(success);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_waypoints_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals) {
	auto fn = [this, state_refs, input_refs, wp_attributes, wp_normals]() {
		std::vector<uint8_t> bytes = WaypointsSrv(state_refs, input_refs, wp_attributes, wp_normals).serialize(tcp_data_types[6]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_start_srv(bool started) {
	auto fn = [this, started]() {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[7];
		full_message[5] = static_cast<uint8_t>(started);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_params(const std::vector<double> &state_refs, const std::vector<double> &attributes) {
	auto fn = [this, state_refs, attributes]() {
		std::vector<uint8_t> bytes = ParamsMsg(state_refs, attributes).serialize(tcp_data_types[8]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	};
	add_stream_task(std::move(fn));
}

void TcpClient::send_run(float v_ref, const std::string &path_name, float x_init, float y_init, float yaw_init) {
	auto fn = [this, v_ref, path_name, x_init, y_init, yaw_init]() {
		std::vector<uint8_t> bytes = RunMsg(v_ref, path_name, x_init, y_init, yaw_init).serialize(tcp_data_types[9]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
		run_sent = true;
	};
	add_stream_task(std::move(fn));
}

// ------------------- //
// UDP Encoding
// ------------------- //

void TcpClient::send_lane2(const utils::Lane2 &lane) {
	auto fn = [this, lane]() {
		std_msgs::Header header = lane.header;
		float center = lane.center;
		int stopline = lane.stopline;
		bool crosswalk = lane.crosswalk;
		std::vector<uint8_t> bytes = Lane2Msg(header, center, stopline, crosswalk, false).serialize(udp_data_types[0]);
		std::vector<uint8_t> segment(MAX_DGRAM, 0);
		std::memcpy(segment.data(), bytes.data(), bytes.size());
		sendto(udp_socket, segment.data(), segment.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	};
	add_dgram_task(std::move(fn));
}

void TcpClient::send_road_object(const std_msgs::Float32MultiArray &array) {
	auto fn = [this, array]() {
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		std::vector<uint8_t> bytes(MAX_DGRAM, 0);
		std::memcpy(bytes.data(), &length, message_size);
		bytes[4] = udp_data_types[1];
		std::memcpy(bytes.data() + header_size, arr.data(), length);
		sendto(udp_socket, bytes.data(), bytes.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	};
	add_dgram_task(std::move(fn));
}

void TcpClient::send_waypoint(const std_msgs::Float32MultiArray &array) {
	auto fn = [this, array]() {
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		std::vector<uint8_t> bytes(MAX_DGRAM, 0);
		std::memcpy(bytes.data(), &length, message_size);
		bytes[4] = udp_data_types[2];
		std::memcpy(bytes.data() + header_size, arr.data(), length);
		sendto(udp_socket, bytes.data(), bytes.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	};
	add_dgram_task(std::move(fn));
}

void TcpClient::send_sign(const std_msgs::Float32MultiArray &array) {
	uint32_t length = ros::serialization::serializationLength(array);
	std::vector<uint8_t> arr(length);
	ros::serialization::OStream stream(arr.data(), length);
	ros::serialization::serialize(stream, array);
	std::vector<uint8_t> bytes(MAX_DGRAM, 0);
	std::memcpy(bytes.data(), &length, message_size);
	bytes[4] = udp_data_types[3];
	std::memcpy(bytes.data() + header_size, arr.data(), length);
	sendto(udp_socket, bytes.data(), bytes.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
}

void TcpClient::send_image_rgb(const cv::Mat &img) {
	std::vector<uchar> image;
	cv::imencode(".jpg", img, image, {cv::IMWRITE_JPEG_QUALITY, 30});
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

void TcpClient::send_image_depth(const cv::Mat &img) {
	std::vector<uchar> image;
	cv::imencode(".png", img, image, {cv::IMWRITE_PNG_COMPRESSION, 4});
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

void TcpClient::send_steer(float steer) {
	auto fn = [this, steer]() {
		uint32_t length = sizeof(steer);
		std::vector<uint8_t> bytes(MAX_DGRAM, 0);
		std::memcpy(bytes.data(), &length, message_size);
		bytes[4] = udp_data_types[6];
		std::memcpy(bytes.data() + header_size, &steer, length);
		sendto(udp_socket, bytes.data(), bytes.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	};
	add_dgram_task(std::move(fn));
}

void TcpClient::send_swload() {
	while (alive) {
		SWLoadMsg msg;
		std_msgs::Float64MultiArray cores_usage = msg.get_cores_usage();
		float ram_usage = msg.get_ram_usage();
		float temp = msg.get_temperature();
		float heap = msg.get_heap_usage();
		float stack = msg.get_stack_usage();
		/* std::cout << "No cores: " << cores_usage.data.size() << std::endl; */
		/* std::cout << "RAM: " << ram_usage << std::endl; */
		/* std::cout << "Temp: " << temp << std::endl; */
		/* std::cout << "Heap use: " << heap << std::endl; */
		/* std::cout << "Stack use: " << stack << std::endl; */
		std::vector<uint8_t> bytes = SWLoadMsg(cores_usage, ram_usage, temp, heap, stack).serialize(udp_data_types[7]);
		std::vector<uint8_t> segment(MAX_DGRAM, 0);
		std::memcpy(segment.data(), bytes.data(), bytes.size());
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
    if (decoded_string == "refresh_run") {
        run_sent = false;
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
