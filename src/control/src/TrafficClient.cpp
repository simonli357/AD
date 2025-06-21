#include "TrafficClient.hpp"
#include "TcpClient.hpp"
#include "utils/constants.h"
#include <arpa/inet.h>
#include <chrono>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <fcntl.h>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <ostream>
#include <ros/ros.h>
#include <sys/socket.h>

using json = nlohmann::json;

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
	main = std::thread(&TrafficClient::initialize, this);
	keyDealer = std::make_unique<KeyDealer>();
	ThreadPools::communication.execute([this] { tasks = std::make_unique<tbb::task_group>(); });
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
		subscribeToLocationData();
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		tcp_can_send = true;
		listen();
	}
}

void TrafficClient::listen() {
	std::array<uint8_t, 1024> buffer;
	while (connected) {
		ssize_t bytes = recv(tcp_socket, buffer.data(), buffer.size(), 0);

		if (bytes > 0) {
			std::cout << "[TrafficClient] received " << bytes << " bytes\n";

			std::cout << "  HEX : ";
			for (ssize_t i = 0; i < bytes; ++i) {
				printf("%02X ", buffer[i]);
			}
			std::cout << '\n';

			std::cout << "  TEXT: ";
			for (ssize_t i = 0; i < bytes; ++i) {
				char c = static_cast<char>(buffer[i]);
				std::cout << (isprint(static_cast<unsigned char>(c)) ? c : '.');
			}
			std::cout << "\n--------------------------------\n";

			// TODO: existing parsing / handling here
		} else if (bytes == 0) {
			connected = false; // connection closed
			break;
		} else { // bytes == -1
			if (errno == EAGAIN || errno == EWOULDBLOCK) {
				usleep(10000); // 10 ms back-off
				continue;
			}
			connected = false;
			break;
		}
	}
	tcp_can_send = false;
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

void TrafficClient::subscribeToLocationData() {
	tasks->run([this] {
		json msg = {{"reqORinfo", "info"}, {"type", "locIDsub"}, {"freq", 0.25}, {"locID", car_id}};
		std::string chars = msg.dump();
		send(tcp_socket, chars.data(), chars.size(), 0);
	});
}

// https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/vehicletoeverything/TrafficCommunication.html
void TrafficClient::send_car_data() {
	if (!can_send()) {
		return;
	}
	tasks->run([this]() {
		std::string v_pos = create_vehicle_pos(Tracking::ego_car->x, Tracking::ego_car->y);
		std::string v_rot = create_vehicle_rot(Tracking::ego_car->yaw);
		std::string v_speed = create_vehicle_speed(Tracking::ego_car->speed);
		std::string msg_string = v_pos + v_rot + v_speed;

		for (auto &obj : Tracking::road_objects) {
			int id = -1;
			if (obj->type == OBJECT::ONEWAY) {
				id = 8;
			} else if (obj->type == OBJECT::NOENTRY) {
				id = 9;
			} else if (obj->type == OBJECT::RAMP) {
				id = 16;
			} else if (obj->type == OBJECT::TUNNEL) {
				id = 16;
			} else if (obj->type == OBJECT::FOG) {
				id = 15;
			}
			if (id > 0)
				msg_string += create_encountered_obstacle(id, obj->x, obj->y);
		}
		for (auto &obj : Tracking::road_known_static_objects) {
			int id = -1;
			if (obj->type == OBJECT::LIGHTS || obj->type == OBJECT::GREENLIGHT || obj->type == OBJECT::YELLOWLIGHT || obj->type == OBJECT::REDLIGHT) {
				auto light_obj = std::dynamic_pointer_cast<Tracking::LightObject>(obj);
				if (!light_obj) {
					continue;
				}
				id = 14;
			} else if (obj->type == OBJECT::HIGHWAYENTRANCE) {
				id = 5;
			} else if (obj->type == OBJECT::STOPSIGN) {
				id = 1;
			} else if (obj->type == OBJECT::ROUNDABOUT) {
				id = 7;
			} else if (obj->type == OBJECT::PARK) {
				id = 3;
			} else if (obj->type == OBJECT::CROSSWALK) {
				id = 4;
			} else if (obj->type == OBJECT::HIGHWAYEXIT) {
				id = 6;
			} else if (obj->type == OBJECT::PRIORITY) {
				id = 2;
			}
			if (id > 0)
				msg_string += create_encountered_obstacle(id, obj->x, obj->y);
		}

		for (auto &car : Tracking::road_cars) {
			int id = 10;
			auto car_obj = std::dynamic_pointer_cast<Tracking::DynamicObject>(car);
			if (!car_obj) {
				continue;
			}
			if (car_obj->parked)
				id = 10;
			if (id > 0)
				msg_string += create_encountered_obstacle(id, car_obj->x, car_obj->y);
		}

		for (auto &car : Tracking::road_pedestrians) {
			int id = 11;
			auto pedestrian_obj = std::dynamic_pointer_cast<Tracking::PedestrianObject>(car);
			if (!pedestrian_obj) {
				continue;
			}
			if (pedestrian_obj->on_crosswalk)
				id = 12;
			if (id > 0)
				msg_string += create_encountered_obstacle(id, pedestrian_obj->x, pedestrian_obj->y);
		}

		send(tcp_socket, msg_string.data(), msg_string.size(), 0);
	});
}
