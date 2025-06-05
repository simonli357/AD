#include "TrafficClient.hpp"
#include "utils/constants.h"
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <memory>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sys/socket.h>

using namespace VehicleConstants;
using json = nlohmann::json;

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
	main = std::thread(&TrafficClient::initialize, this);
	keyDealer = std::make_unique<KeyDealer>();
	create_udp_socket();
	ThreadPools::communication.execute([this] { tasks = std::make_unique<tbb::task_group>(); });
    receive_datagram();
}

TrafficClient::~TrafficClient() {
	alive = false;
	connected = false;
	if (tcp_socket != -1) {
		close(tcp_socket);
	}
	if (main.joinable()) {
		main.join();
	}
}

// ------------------- //
// Utility Methods
// ------------------- //

void TrafficClient::create_tcp_socket() {
	tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	tcp_address.sin_family = AF_INET;
	tcp_address.sin_port = htons(tcp_port);
	inet_pton(AF_INET, server_address.c_str(), &tcp_address.sin_addr);
	int flags = fcntl(tcp_socket, F_GETFL, 0);
	fcntl(tcp_socket, F_SETFL, flags | O_NONBLOCK);
}

void TrafficClient::create_udp_socket() {
    try {
        udp_socket = std::make_unique<ip::udp::socket>(udp_io_ctx);
        udp_socket->open(ip::udp::v4());
        udp_socket->set_option(socket_base::reuse_address(true));
        udp_socket->bind(ip::udp::endpoint(ip::udp::v4(), udp_port));
        ROS_INFO("UDP socket bound to port: %d", udp_socket->local_endpoint().port());
        receive_datagram();
    } catch (const std::exception& e) {
        ROS_ERROR("Failed to create UDP socket: %s", e.what());
    }
}

void TrafficClient::initialize() {
	while (alive) {
		create_tcp_socket();
		std::cout << "Connecting to Traffic Server \n" << std::endl;
		while (true) {
			if (connect(tcp_socket, (struct sockaddr *)&tcp_address, sizeof(tcp_address)) != -1) {
				break;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
		}
		std::cout << "Successfully connected to Traffic Server \n" << std::endl;
		connected = true;
		send_car_id();
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		tcp_can_send = true;
		poll_connection();
	}
}

void TrafficClient::poll_connection() {
	while (alive) {
		char buffer[32];
		if (connected && recv(tcp_socket, buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT) == 0) {
			std::cout << "Traffic Server Disconnected \n" << std::endl;
			connected = false;
			tcp_can_send = false;
			close(tcp_socket);
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
}

void TrafficClient::receive_datagram() {
	udp_socket->async_receive_from(boost::asio::buffer(udp_recv_buffer), remote_endpoint, [this](boost::system::error_code ec, std::size_t bytes_recvd) { this->on_datagram(ec, bytes_recvd); });
}

void TrafficClient::on_datagram(const boost::system::error_code &error, std::size_t bytes_transferred) {
    if (!error) {
        // Process datagram here (bytes_transferred bytes in recv_buffer_)
        std::cout << "Received: " << std::string(udp_recv_buffer.data(), bytes_transferred) << "\n";
    } else {
        std::cerr << "Receive error: " << error.message() << "\n";
    }
    receive_datagram();
}

bool TrafficClient::can_send() {
	auto now = steady_clock::now();
	auto elapsed = duration_cast<milliseconds>(now - last_send_time);
	if (elapsed.count() >= 250) {
		last_send_time = now;
		return true;
	}
	return false;
}

std::string TrafficClient::create_vehicle_pos(double x, double y) {
	json msg = {{"reqORinfo", "info"}, {"type", "devicePos"}, {"value1", x}, {"value2", y}};
	return msg.dump();
}

std::string TrafficClient::create_vehicle_rot(double yaw) {
	json msg = {{"reqORinfo", "info"}, {"type", "deviceRot"}, {"value1", yaw}};
	return msg.dump();
}

std::string TrafficClient::create_vehicle_speed(double speed) {
	json msg = {{"reqORinfo", "info"}, {"type", "deviceSpeed"}, {"value1", speed}};
	return msg.dump();
}

std::string TrafficClient::create_encountered_obstacle(int type, double x, double y) {
	json msg = {{"reqORinfo", "info"}, {"type", "historyData"}, {"value1", type}, {"value2", x}, {"value3", y}};
	return msg.dump();
}

// ------------------- //
// TCP Encoding
// ------------------- //

void TrafficClient::send_car_id() {
	tasks->run([this] {
		json msg = {{"reqORinfo", "info"}, {"type", "locIDsub"}, {"freq", 0.25}, {"locID", car_id}};
		std::string chars = msg.dump();
		send(tcp_socket, chars.data(), chars.size(), 0);
	});
}

// https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/vehicletoeverything/TrafficCommunication.html
void TrafficClient::send_car_data(const Float32MultiArray &road_object) {
	if (!can_send()) {
		return;
	}
	static auto road_obj_to_int = [](OBJECT &obj) -> int {
		switch (obj) {
		case OBJECT::BLOCK:
			return 13;
		case OBJECT::CAR:
			return -1;
		case OBJECT::CROSSWALK:
			return 4;
		case OBJECT::GREENLIGHT:
			return 14;
		case OBJECT::HIGHWAYENTRANCE:
			return 5;
		case OBJECT::HIGHWAYEXIT:
			return 6;
		case OBJECT::LIGHTS:
			return 14;
		case OBJECT::NOENTRY:
			return 9;
		case OBJECT::NONE:
			return -1;
		case OBJECT::ONEWAY:
			return 8;
		case OBJECT::PARK:
			return 3;
		case OBJECT::PEDESTRIAN:
			return 12;
		case OBJECT::PRIORITY:
			return 2;
		case OBJECT::REDLIGHT:
			return 14;
		case OBJECT::ROUNDABOUT:
			return 7;
		case OBJECT::STOPSIGN:
			return 1;
		case OBJECT::YELLOWLIGHT:
			return 14;
		default:
			return -1;
		}
	};
	tasks->run([this, road_object]() {
        int num_objects = road_object.data.size() / 8;
        if (road_object.data.size() % 8 != 0) return;

        double x = road_object.data[1];
        double y = road_object.data[2];
        double rot = road_object.data[3];
        double speed = road_object.data[4];

		std::string v_pos = create_vehicle_pos(x, y);
		std::string v_rot = create_vehicle_rot(rot);
		std::string v_speed = create_vehicle_speed(speed);
        
        std::string objcts = "";
        for (int i = 1; i < num_objects; ++i) {
            int type = road_object.data[i * 8];
            double x = road_object.data[i * 8 + 1];
            double y = road_object.data[i * 8 + 2];
			OBJECT obj_type = static_cast<OBJECT>(static_cast<int>(type));
			objcts += create_encountered_obstacle(road_obj_to_int(obj_type), x, y);
        }

        std::string msg = v_pos + v_rot + v_speed + objcts;
        send(tcp_socket, msg.data(), msg.size(), 0);
    });
}
