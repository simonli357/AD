#include "TrafficClient.hpp"
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
#include <ros/ros.h>
#include <sys/socket.h>

using namespace VehicleConstants;
using json = nlohmann::json;

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
    poll = std::thread(&TrafficClient::initialize, this); 
    sender = std::thread(&TrafficClient::send_data, this);
}

TrafficClient::~TrafficClient() {
	alive = false;
	connected = false;
	if (tcp_socket != -1) {
		close(tcp_socket);
	}
	if (poll.joinable()) {
		poll.join();
	}
	if (sender.joinable()) {
		sender.join();
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

void TrafficClient::send_car_data2() {
	std::string v_pos = create_vehicle_pos(Tracking::ego_car->x, Tracking::ego_car->y);
	std::string v_rot = create_vehicle_rot(Tracking::ego_car->yaw);
	std::string v_speed = create_vehicle_speed(Tracking::ego_car->speed);
	std::string msg_string = v_pos + v_rot + v_speed;

	for (auto& obj: Tracking::road_objects) {
		int id = -1;
		if (obj->type == OBJECT::ONEWAY) {
			id = 8;
		} else if (obj->type == OBJECT::NOENTRY) {
			id = 9;
		} else if (obj->type == OBJECT::RAMP) {
			id = -4;
		} else if (obj->type == OBJECT::TUNNEL) {
			id = -5;
		} else if (obj->type == OBJECT::FOG) {
			id = -6;
		}
		if (id > 0) msg_string += create_encountered_obstacle(id, obj->x, obj->y);
	}
	for (auto& obj: Tracking::road_known_static_objects) {
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
		if (id > 0) msg_string += create_encountered_obstacle(id, obj->x, obj->y);
	}

	for (auto& car: Tracking::road_cars) {
		int id = -1;
		auto car_obj = std::dynamic_pointer_cast<Tracking::DynamicObject>(car);
		if (!car_obj) {
				continue;
		}
		if (car_obj->parked) id = 14;
		if (id > 0) msg_string += create_encountered_obstacle(id, car_obj->x, car_obj->y);
	}

	for (auto& car: Tracking::road_pedestrians) {
		int id = -2;
		auto pedestrian_obj = std::dynamic_pointer_cast<Tracking::PedestrianObject>(car);
		if (!pedestrian_obj) {
				continue;
		}
		if (pedestrian_obj->on_crosswalk) id = -3;
		if (id > 0) msg_string += create_encountered_obstacle(id, pedestrian_obj->x, pedestrian_obj->y);
	}

	auto fn = [this, msg_string]() {
		send(tcp_socket, msg_string.data(), msg_string.size(), 0);
	};
	add_stream_task(std::move(fn));
}
void TrafficClient::send_data() {
	while (alive) {
		if (!stream_tasks.empty() && tcp_can_send) {
            std::any stream_task;
            if (stream_tasks.try_pop(stream_task)) {
                std::function<void()> task = std::any_cast<std::function<void()>>(stream_task);
                task();
            }
            stream_tasks.clear();
		}
		std::this_thread::sleep_for(std::chrono::milliseconds(frequency));
	}
}

template <typename Callable> void TrafficClient::add_stream_task(Callable &&lambda) { stream_tasks.push(std::function<void()>(std::forward<Callable>(lambda))); }

// ------------------- //
// TCP Encoding
// ------------------- //

void TrafficClient::send_car_id() {
	json msg = {{"reqORinfo", "info"}, {"type", "locIDsub"}, {"freq", 0.25}, {"locID", car_id}};
	std::string chars = msg.dump();
	send(tcp_socket, chars.data(), chars.size(), 0);
}

// https://bosch-future-mobility-challenge-documentation.readthedocs-hosted.com/data/vehicletoeverything/TrafficCommunication.html
void TrafficClient::send_car_data(const Float32MultiArray &road_object) {
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
    auto fn = [this, road_object]() {
		std::string v_pos = create_vehicle_pos(road_object.data[1], road_object.data[2]);
		std::string v_rot = create_vehicle_rot(road_object.data[3]);
		std::string v_speed = create_vehicle_speed(road_object.data[4]);
        std::string msg_string = "";
		for (size_t i = 7; i < road_object.data.size(); i += 7) {
			OBJECT obj_type = static_cast<OBJECT>(static_cast<int>(road_object.data[i]));
			msg_string += create_encountered_obstacle(road_obj_to_int(obj_type), road_object.data[i + 1], road_object.data[i + 2]);
		}
        std::string msg = v_pos + v_rot + v_speed + msg_string;
        send(tcp_socket, msg.data(), msg.size(), 0);
    };
    add_stream_task(std::move(fn));
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