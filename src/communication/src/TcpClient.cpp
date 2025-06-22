#include "TcpClient.hpp"
#include "ros/serialization.h"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Header.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "utils/Lane2.h"
#include "utils/constants.h"
#include <algorithm>
#include <arpa/inet.h>
#include <chrono>
#include <cstdint>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <hwloc.h>
#include <iostream>
#include <memory>
#include <netinet/in.h>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <ros/ros.h>
#include <sys/socket.h>
#include <thread>
#include <utility>
#include <utils/constants.h>
#include <vector>

TcpClient::TcpClient(bool use_tcp, const std::string client_type, const std::string ip_address) : client_type(client_type), server_address(ip_address) {
	swload = std::make_unique<SWLoadMsg>();
	lane2_msg = std::make_unique<Lane2Msg>();
	params_msg = std::make_unique<ParamsMsg>();
	run_msg = std::make_unique<RunMsg>();
	trigger_msg = std::make_unique<TriggerMsg>();
    imu_msg = std::make_unique<ImuMsg>();

	goto_cmd_srv = std::make_unique<GoToCmdSrv>();
	goto_srv = std::make_unique<GoToSrv>();
	waypoints_srv = std::make_unique<WaypointsSrv>();
	set_states_srv = std::make_unique<SetStatesSrv>();

	create_udp_socket();
	set_udp_data_types();

	ThreadPools::communication.execute([this] { tasks = std::make_unique<tbb::task_group>(); });

	if (use_tcp) {
		set_tcp_data_types();
		set_tcp_data_actions();
		main = std::thread(&TcpClient::run, this);
	}
}

TcpClient::~TcpClient() {
	alive = false;
	connected = false;
	if (tcp_socket != -1) {
		close(tcp_socket);
	}
	if (main.joinable()) {
		main.join();
	}
	tasks->wait();
}

// ------------------- //
// Utility Methods
// ------------------- //

void TcpClient::create_tcp_socket() {
	tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	tcp_address.sin_family = AF_INET;
	tcp_address.sin_port = htons(tcp_port);
	setsockopt(tcp_socket, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
	setsockopt(tcp_socket, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
	inet_pton(AF_INET, server_address.c_str(), &tcp_address.sin_addr);
	int flags = fcntl(tcp_socket, F_GETFL, 0);
	fcntl(tcp_socket, F_SETFL, flags | O_NONBLOCK);
}

void TcpClient::create_udp_socket() {
	udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	udp_address.sin_family = AF_INET;
	udp_address.sin_port = htons(udp_port);
	setsockopt(udp_socket, SOL_SOCKET, SO_SNDBUF, &buffer_size, sizeof(buffer_size));
	setsockopt(udp_socket, SOL_SOCKET, SO_RCVBUF, &buffer_size, sizeof(buffer_size));
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
	tcp_data_types.push_back(0x0b); // Cardinal yaw reset
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
	udp_data_types.push_back(0x09); // ModelStates
    udp_data_types.push_back(0x0a); // Imu Calibration
}

void TcpClient::set_tcp_data_actions() {
	tcp_data_actions[tcp_data_types[0]] = &TcpClient::parse_string;			// std::string
	tcp_data_actions[tcp_data_types[1]] = &TcpClient::parse_trigger_msg;	// Trigger
	tcp_data_actions[tcp_data_types[4]] = &TcpClient::parse_go_to_cmd_srv;	// GoToCmdSrc
	tcp_data_actions[tcp_data_types[5]] = &TcpClient::parse_set_states_srv; // SetStatesSrv
	tcp_data_actions[tcp_data_types[6]] = &TcpClient::parse_waypoints_srv;	// SetStatesSrv
	tcp_data_actions[tcp_data_types[7]] = &TcpClient::parse_start_srv;		// StartSrv
	tcp_data_actions[tcp_data_types[10]] = &TcpClient::parse_yaw;			// Cardinal yaw reset
}

void TcpClient::run() {
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
		if (!client_type.empty()) {
			send_type(client_type);
		}
		connected = true;
		tcp_can_send = true;
		listen();
	}
}

void TcpClient::listen() {
	std::array<uint8_t, 5> header_buffer;
	while (connected) {
		// --- Header Reception ---
		ssize_t total_header_received = 0;
		while (total_header_received < 5) {
			if (++swload_counter == 50) {
				send_swload();
				swload_counter = 0;
			}

			// --- Read data ---
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
	tcp_can_send = false;
}

// ------------------- //
// TCP Encoding
// ------------------- //

void TcpClient::send_type(const std::string &str) {
	tasks->run([this, str] {
		uint32_t length = str.size();
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[0];
		std::memcpy(full_message.data() + header_size, str.data(), length);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	});
}

void TcpClient::send_string(const std::string &str) {
	tasks->run([this, str] {
		uint32_t length = str.size();
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[0];
		std::memcpy(full_message.data() + header_size, str.data(), length);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	});
}

void TcpClient::send_string(const std::string &str, uint8_t datatype) {
	tasks->run([this, str, datatype] {
		uint32_t length = str.size();
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = datatype;
		std::memcpy(full_message.data() + header_size, str.data(), length);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	});
}

void TcpClient::send_trigger(const std_srvs::Trigger &trigger) {
	tasks->run([this, trigger] {
		trigger_msg->encode(trigger);
		std::vector<uint8_t> bytes = trigger_msg->serialize(tcp_data_types[1]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	});
}

void TcpClient::send_message(const std_msgs::String &msg) {
	tasks->run([this, msg] {
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
	});
}

void TcpClient::send_go_to_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals) {
	tasks->run([this, state_refs, input_refs, wp_attributes, wp_normals] {
		goto_srv->encode(state_refs, input_refs, wp_attributes, wp_normals);
		std::vector<uint8_t> bytes = goto_srv->serialize(tcp_data_types[3]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	});
}

void TcpClient::send_go_to_cmd_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals, bool success) {
	tasks->run([this, state_refs, input_refs, wp_attributes, wp_normals, success] {
		goto_cmd_srv->encode(state_refs, input_refs, wp_attributes, wp_normals, success);
		std::vector<uint8_t> bytes = goto_cmd_srv->serialize(tcp_data_types[4]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	});
}

void TcpClient::send_set_states_srv(bool success) {
	tasks->run([this, success] {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[5];
		full_message[5] = static_cast<uint8_t>(success);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	});
}

void TcpClient::send_waypoints_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals) {
	tasks->run([this, state_refs, input_refs, wp_attributes, wp_normals] {
		waypoints_srv->encode(state_refs, input_refs, wp_attributes, wp_normals);
		std::vector<uint8_t> bytes = waypoints_srv->serialize(tcp_data_types[6]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	});
}

void TcpClient::send_start_srv(bool started) {
	tasks->run([this, started] {
		uint32_t length = 1;
		size_t total_size = header_size + length;
		std::vector<uint8_t> full_message(total_size);
		std::memcpy(full_message.data(), &length, message_size);
		full_message[4] = tcp_data_types[7];
		full_message[5] = static_cast<uint8_t>(started);
		send(tcp_socket, full_message.data(), full_message.size(), 0);
	});
}

void TcpClient::send_params(const std::vector<double> &state_refs, const std::vector<double> &attributes) {
	tasks->run([this, state_refs, attributes] {
		params_msg->encode(state_refs, attributes);
		std::vector<uint8_t> bytes = params_msg->serialize(tcp_data_types[8]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	});
}

void TcpClient::send_run(float v_ref, const std::string &path_name, float x_init, float y_init, float yaw_init) {
	tasks->run([this, v_ref, path_name, x_init, y_init, yaw_init] {
		run_msg->encode(v_ref, path_name, x_init, y_init, yaw_init);
		std::vector<uint8_t> bytes = run_msg->serialize(tcp_data_types[9]);
		send(tcp_socket, bytes.data(), bytes.size(), 0);
	});
}

// ------------------- //
// UDP Encoding
// ------------------- //

void TcpClient::send_lane2(const utils::Lane2 &lane) {
	tasks->run([this, lane] {
		auto &udp_buffer = udp_buffers.local();
		std_msgs::Header header = lane.header;
		float center = lane.center;
		bool stopline = lane.stopline;
		float stopline_dist = lane.stopline_dist;
		bool crosswalk = lane.crosswalk;
		lane2_msg->encode(header, center, stopline, stopline_dist, crosswalk, false);
		std::vector<uint8_t> bytes = lane2_msg->serialize(udp_data_types[0]);
		std::memcpy(udp_buffer.data(), bytes.data(), bytes.size());
		sendto(udp_socket, udp_buffer.data(), udp_buffer.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

void TcpClient::send_road_object(const std_msgs::Float32MultiArray &array) {
	tasks->run([this, array] {
		auto &udp_buffer = udp_buffers.local();
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		std::memcpy(udp_buffer.data(), &length, message_size);
		udp_buffer[4] = udp_data_types[1];
		std::memcpy(udp_buffer.data() + header_size, arr.data(), length);
		sendto(udp_socket, udp_buffer.data(), udp_buffer.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

void TcpClient::send_waypoint(const std_msgs::Float32MultiArray &array) {
	tasks->run([this, array] {
		auto &udp_buffer = udp_buffers.local();
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		std::memcpy(udp_buffer.data(), &length, message_size);
		udp_buffer[4] = udp_data_types[2];
		std::memcpy(udp_buffer.data() + header_size, arr.data(), length);
		sendto(udp_socket, udp_buffer.data(), udp_buffer.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

void TcpClient::send_sign(const std::vector<float> &data) {
	tasks->run([this, data] {
		auto &udp_buffer = udp_buffers.local();
		std_msgs::Float32MultiArray array;
		array.data = std::move(data);
		uint32_t length = ros::serialization::serializationLength(array);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, array);
		std::memcpy(udp_buffer.data(), &length, message_size); // message_size = sizeof(uint32_t)
		udp_buffer[4] = udp_data_types[3];					   // Data type marker
		std::memcpy(udp_buffer.data() + header_size, arr.data(), length);
		sendto(udp_socket, udp_buffer.data(), header_size + length, 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

void TcpClient::send_image_rgb(const cv::Mat &img) {
	tasks->run([this, img] {
		auto &udp_buffer = udp_buffers.local();
		auto &image_buffer = image_buffers.local();
		cv::imencode(".jpg", img, image_buffer, {cv::IMWRITE_JPEG_QUALITY, rgb_img_quality});
		uint32_t length = image_buffer.size();
		size_t payload_size = MAX_DGRAM - header_size;
		uint8_t total_segments = std::ceil(static_cast<float>(length + header_size) / MAX_DGRAM);
		for (uint8_t seg_num = 0; seg_num < total_segments; ++seg_num) {
			udp_buffer[0] = total_segments;
			udp_buffer[1] = seg_num;

			size_t start = seg_num * payload_size;
			size_t end = std::min(start + payload_size, static_cast<size_t>(length));
			uint16_t chunk_size = end - start;
			std::memcpy(udp_buffer.data() + 2, &chunk_size, 2);

			udp_buffer[4] = udp_data_types[4];

			std::memcpy(udp_buffer.data() + header_size, image_buffer.data() + start, chunk_size);
			sendto(udp_socket, udp_buffer.data(), header_size + chunk_size, 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
		}
	});
}

void TcpClient::send_image_depth(const cv::Mat &img) {
	tasks->run([this, img] {
		auto &udp_buffer = udp_buffers.local();
		auto &depth_buffer = image_buffers.local();
		cv::imencode(".png", img, depth_buffer, {cv::IMWRITE_PNG_COMPRESSION, 3});
		uint32_t length = depth_buffer.size();
		size_t payload_size = MAX_DGRAM - header_size;
		uint8_t total_segments = std::ceil(static_cast<float>(length + header_size) / MAX_DGRAM);
		for (uint8_t seg_num = 0; seg_num < total_segments; ++seg_num) {
			udp_buffer[0] = total_segments;
			udp_buffer[1] = seg_num;

			size_t start = seg_num * payload_size;
			size_t end = std::min(start + payload_size, static_cast<size_t>(length));
			uint16_t chunk_size = end - start;
			std::memcpy(udp_buffer.data() + 2, &chunk_size, 2);

			udp_buffer[4] = udp_data_types[5];

			std::memcpy(udp_buffer.data() + header_size, depth_buffer.data() + start, chunk_size);
			sendto(udp_socket, udp_buffer.data(), header_size + chunk_size, 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
		}
	});
}

void TcpClient::send_steer(float steer) {
	tasks->run([this, steer] {
		auto &udp_buffer = udp_buffers.local();
		uint32_t length = sizeof(steer);
		std::memcpy(udp_buffer.data(), &length, message_size);
		udp_buffer[4] = udp_data_types[6];
		std::memcpy(udp_buffer.data() + header_size, &steer, length);
		sendto(udp_socket, udp_buffer.data(), udp_buffer.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

void TcpClient::send_swload() {
	tasks->run([this] {
		auto &udp_buffer = udp_buffers.local();
		swload->refresh();
		std::vector<uint8_t> bytes = swload->serialize(udp_data_types[7]);
		std::memcpy(udp_buffer.data(), bytes.data(), bytes.size());
		sendto(udp_socket, udp_buffer.data(), udp_buffer.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

void TcpClient::send_model_states(const geometry_msgs::Pose &msg) {
	tasks->run([this, msg] {
		auto &udp_buffer = udp_buffers.local();
		uint32_t length = ros::serialization::serializationLength(msg);
		std::vector<uint8_t> arr(length);
		ros::serialization::OStream stream(arr.data(), length);
		ros::serialization::serialize(stream, msg);
		std::memcpy(udp_buffer.data(), &length, message_size);
		udp_buffer[4] = udp_data_types[8];
		std::memcpy(udp_buffer.data() + header_size, arr.data(), length);
		sendto(udp_socket, udp_buffer.data(), udp_buffer.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

void TcpClient::send_imu_calib(float sys_calib, float gyro_calib, float mag_calib, float accel_calib) {
	tasks->run([this, sys_calib, gyro_calib, mag_calib, accel_calib] {
		auto &udp_buffer = udp_buffers.local();
		imu_msg->encode(sys_calib, gyro_calib, mag_calib, accel_calib);
		std::vector<uint8_t> bytes = imu_msg->serialize(udp_data_types[9]);
		std::memcpy(udp_buffer.data(), bytes.data(), bytes.size());
		sendto(udp_socket, udp_buffer.data(), udp_buffer.size(), 0, (struct sockaddr *)&udp_address, sizeof(udp_address));
	});
}

// ------------------- //
// TCP Decoding
// ------------------- //

void TcpClient::parse_string(std::vector<uint8_t> &bytes) {
	std::string decoded_string(bytes.begin(), bytes.end());
	if (decoded_string == "ack") {
		tcp_can_send = true;
		if (!ack_callback) {
			return;
		}
		ack_callback();
		std::cout << client_type << " successfully connected to GUI.\n" << std::endl;
		return;
	}
	if (decoded_string == "refresh_run") {
		if (!send_run_callback || !tcp_can_send) {
			return;
		}
		send_run_callback();
		std::cout << "Resending Run Parameters to GUI" << std::endl;
		return;
	}
}

void TcpClient::parse_trigger_msg(std::vector<uint8_t> &bytes) {
	if (!trigger_response_callback) {
		return;
	}
	trigger_msg->deserialize(bytes);
	trigger_response_callback(trigger_msg->response);
}

void TcpClient::parse_go_to_cmd_srv(std::vector<uint8_t> &bytes) {
	if (!go_to_cmd_callback) {
		return;
	}
	goto_cmd_srv->deserialize(bytes);
	go_to_cmd_callback(goto_cmd_srv->coords);
}

void TcpClient::parse_set_states_srv(std::vector<uint8_t> &bytes) {
	if (!set_states_callback) {
		return;
	}
	set_states_srv->deserialize(bytes);
	set_states_callback(set_states_srv->x, set_states_srv->y);
}

void TcpClient::parse_waypoints_srv(std::vector<uint8_t> &bytes) {
	if (!waypoints_callback) {
		return;
	}
	waypoints_srv->deserialize(bytes);
	waypoints_callback(waypoints_srv->x0, waypoints_srv->y0, waypoints_srv->yaw0);
}

void TcpClient::parse_start_srv(std::vector<uint8_t> &bytes) {
	if (!start_callback) {
		return;
	}
	std::string decoded_string(bytes.begin(), bytes.end());
	if (decoded_string == "start") {
		start_callback(true);
	} else if (decoded_string == "stop") {
		start_callback(false);
	}
}

void TcpClient::parse_yaw(std::vector<uint8_t> &bytes) {
	if (!yaw_callback) {
		return;
	}
	int direction;
	std::memcpy(&direction, bytes.data(), 4);
	/* switch (direction) { */
	/* case 0: */
	/* 	std::cout << "Setting orientation to EAST" << std::endl; */
	/* 	break; // north */
	/* case 1: */
	/* 	std::cout << "Setting orientation to NORTH" << std::endl; */
	/* 	break; // east */
	/* case 2: */
	/* 	std::cout << "Setting orientation to WEST" << std::endl; */
	/* 	break; // south */
	/* case 3: */
	/* 	std::cout << "Setting orientation to SOUTH" << std::endl; */
	/* 	break; // west */
	/* default: */
	/* 	return; // invalid input, do nothing */
	/* } */
	yaw_callback(direction);
}
