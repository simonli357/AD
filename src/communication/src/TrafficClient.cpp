#include "TrafficClient.hpp"
#include <arpa/inet.h>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <ros/ros.h>
#include <sys/socket.h>
#include "utils/constants.h"

using namespace VehicleConstants;
using json = nlohmann::json;

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
	receive = std::thread(&TrafficClient::initialize, this);
	poll = std::thread(&TrafficClient::poll_connection, this);
}

TrafficClient::~TrafficClient() {
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

void TrafficClient::create_tcp_socket() {
	tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	tcp_address.sin_family = AF_INET;
	tcp_address.sin_port = htons(tcp_port);
	inet_pton(AF_INET, server_address.c_str(), &tcp_address.sin_addr);
	int flags = fcntl(tcp_socket, F_GETFL, 0);
	fcntl(tcp_socket, F_SETFL, flags | O_NONBLOCK);
}

void TrafficClient::initialize() {
	create_tcp_socket();
	std::cout << "Connecting to Traffic Server \n" << std::endl;
	while (true) {
		if (connect(tcp_socket, (struct sockaddr *)&tcp_address, sizeof(tcp_address)) != -1) {
			break;
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(500));
	}
	std::cout << "Connection request established with Traffic Server \n" << std::endl;
	connected = true;
	std::this_thread::sleep_for(std::chrono::milliseconds(500));
	tcp_can_send = true;
	listen();
}

void TrafficClient::poll_connection() {
	while (alive) {
		char buffer[32];
		if (connected && recv(tcp_socket, buffer, sizeof(buffer), MSG_PEEK | MSG_DONTWAIT) == 0) {
			std::cout << "Traffic Server Disconnected \n" << std::endl;
			connected = false;
			tcp_can_send = false;
			pthread_cancel(receive.native_handle());
			if (receive.joinable()) {
				receive.join();
			}
			close(tcp_socket);
			receive = std::thread(&TrafficClient::initialize, this);
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(250));
	}
}

void TrafficClient::listen() {
	std::string buffer;
	char temp[buffer_size];

	while (connected) {
		ssize_t bytes_received = recv(tcp_socket, temp, sizeof(temp), 0);
		if (bytes_received <= 0)
			break;

		buffer.append(temp, bytes_received);
		size_t processed = 0;
		while (processed < buffer.size()) {
			try {
				json msg = json::parse(buffer.substr(processed));
				if (msg.contains("error")) {
					std::cerr << "Server error: " << msg["error"] << std::endl;
					processed += msg.dump().size();
					continue;
				}
				processed += msg.dump().size();
			} catch (const json::parse_error &e) {
				break;
			}
		}
		buffer.erase(0, processed);
	}
}

// ------------------- //
// TCP Encoding
// ------------------- //

void TrafficClient::send_car_data(const Float32MultiArray &road_object) {
    static auto road_obj_to_str = [](OBJECT &obj) -> std::string {
        switch (obj) {
            case OBJECT::BLOCK: return "BLOCK";
            case OBJECT::CAR: return "CAR";
            case OBJECT::CROSSWALK: return "CROSSWALK";
            case OBJECT::GREENLIGHT: return "GREENLIGHT";
            case OBJECT::HIGHWAYENTRANCE: return "HIGHWAYENTRANCE";
            case OBJECT::HIGHWAYEXIT: return "HIGHWAYEXIT";
            case OBJECT::LIGHTS: return "LIGHTS";
            case OBJECT::NOENTRY: return "NOENTRY";
            case OBJECT::NONE: return "NONE";
            case OBJECT::ONEWAY: return "ONEWAY";
            case OBJECT::PARK: return "PARK";
            case OBJECT::PEDESTRIAN: return "PEDESTRIAN";
            case OBJECT::PRIORITY: return "PRIORITY";
            case OBJECT::REDLIGHT: return "REDLIGHT";
            case OBJECT::ROUNDABOUT: return "ROUNDABOUT";
            case OBJECT::STOPSIGN: return "STOPSIGN";
            case OBJECT::YELLOWLIGHT: return "YELLOWLIGHT";
            default: return "UNKNOWN";
        }
    };

	send_vehicle_pos(road_object.data[1], road_object.data[2]);
	send_vehicle_rot(road_object.data[3]);
	send_vehicle_speed(road_object.data[4]);

	for (size_t i = 7; i < road_object.data.size(); i += 7) {
        OBJECT obj_type = static_cast<OBJECT>(static_cast<int>(road_object.data[i]));
		send_encountered_obstacle(road_obj_to_str(obj_type), road_object.data[i + 1], road_object.data[i + 2]);
	}
}

void TrafficClient::send_vehicle_pos(double x, double y) {
    json msg = {
        {"reqORinfo", "info"},
        {"type", "devicePos"},
        {"value1", x},
        {"value2", y}
    };
    send(tcp_socket, msg.dump().data(), msg.dump().size(), 0);
}

void TrafficClient::send_vehicle_rot(double yaw) {
    json msg = {
        {"reqORinfo", "info"},
        {"type", "deviceRot"},
        {"value1", yaw}
    };
    send(tcp_socket, msg.dump().data(), msg.dump().size(), 0);
}

void TrafficClient::send_vehicle_speed(double speed) {
    json msg = {
        {"reqORinfo", "info"},
        {"type", "deviceSpeed"},
        {"value1", speed}
    };
    send(tcp_socket, msg.dump().data(), msg.dump().size(), 0);
}

void TrafficClient::send_encountered_obstacle(const std::string &type, double x, double y) {
    json msg = {
        {"reqORinfo", "info"},
        {"type", "historyData"},
        {"value1", type},
        {"value2", x},
        {"value3", y}
    };
    send(tcp_socket, msg.dump().data(), msg.dump().size(), 0);
}
