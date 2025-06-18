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
#include <ostream>
#include <ros/ros.h>
#include <sys/socket.h>

using json = nlohmann::json;

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
	main = std::thread(&TrafficClient::initialize, this);
	keyDealer = std::make_unique<KeyDealer>();
	ThreadPools::communication.execute([this] { tasks = std::make_unique<tbb::task_group>(); });
	// create_udp_socket();
	// receive_datagram();
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
		udp_socket->bind(ip::udp::endpoint(ip::address_v4::any(), udp_port));
		ROS_INFO("UDP socket bound to port: %d", udp_socket->local_endpoint().port());
		receive_datagram();
	} catch (const std::exception &e) {
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
void TrafficClient::send_car_data() {
	if (!can_send()) {
		return;
	}
	tasks->run([this]() {
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
                id = 16;
            } else if (obj->type == OBJECT::TUNNEL) {
                id = 16;
            } else if (obj->type == OBJECT::FOG) {
                id = 15;
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
            int id = 10;
            auto car_obj = std::dynamic_pointer_cast<Tracking::DynamicObject>(car);
            if (!car_obj) {
                    continue;
            }
            if (car_obj->parked) id = 10;
            if (id > 0) msg_string += create_encountered_obstacle(id, car_obj->x, car_obj->y);
        }

        for (auto& car: Tracking::road_pedestrians) {
            int id = 11;
            auto pedestrian_obj = std::dynamic_pointer_cast<Tracking::PedestrianObject>(car);
            if (!pedestrian_obj) {
                    continue;
            }
            if (pedestrian_obj->on_crosswalk) id = 12;
            if (id > 0) msg_string += create_encountered_obstacle(id, pedestrian_obj->x, pedestrian_obj->y);
        }

        std::cout << "Sending data to Traffic Server: " << msg_string << std::endl;
        auto fn = [this, msg_string]() {
            send(tcp_socket, msg_string.data(), msg_string.size(), 0);
        };
	});
}
