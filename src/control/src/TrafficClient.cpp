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
#include <tuple>

using json = nlohmann::json;

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
	main = std::thread(&TrafficClient::initialize, this);
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
        if (enough_points) {
            continue;
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
        }

		ssize_t bytes = recv(tcp_socket, buffer.data(), buffer.size(), 0);

		if (bytes > 0) {
			uint8_t *begin = buffer.data();
			uint8_t *end = buffer.data() + bytes;
			uint8_t *json_start = std::find(begin, end, '{');

			if (json_start == end) {
				continue;
			}

			std::string_view json_view(reinterpret_cast<char *>(json_start), end - json_start);

			if (!nlohmann::json::accept(json_view)) {
				continue;
			}

			nlohmann::json msg = nlohmann::json::parse(json_view);

			double x = msg.value("x", 0.0);
			double y = msg.value("y", 0.0);
			double z = msg.value("z", 0.0);

			std::cout << "GPS DATA: " << msg.dump() << std::endl;

			handle_location_data(x / 1000, y / 1000, z / 1000);
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

void TrafficClient::handle_location_data(double x, double y, double z) {
	car_positions[array_ptr] = {x, y};
	array_ptr = (array_ptr + 1) % car_positions.size();
    if (array_ptr == 0) {
        enough_points = true;
    }
}

void TrafficClient::clear_positions() {
    car_positions = std::array<std::pair<double, double>, 32>{};
    array_ptr = 0;
}

void TrafficClient::get_car_position(double &out_x, double &out_y) {
	constexpr size_t TARGET_SAMPLES = 32;
	constexpr double MAX_ACCEPTABLE_STD = 0.25;
	constexpr double CLUSTER_RADIUS = 0.3;
	constexpr size_t MIN_CLUSTER_SIZE = 6;

    if (!enough_points) {
        out_x = std::nan("");
        out_y = std::nan("");
        return;
    } 

	// Statistical filtering
	auto [mean_x, mean_y] = calculate_mean(car_positions);
	auto [std_x, std_y] = calculate_std_dev(car_positions, mean_x, mean_y);

	// First pass outlier removal
	auto filtered = filter_outliers(car_positions, mean_x, mean_y, std_x, std_y, 2.0);

	// Density-based clustering
	auto clusters = cluster_points(filtered, CLUSTER_RADIUS);
	if (clusters.empty()) {
		std::cout << "No valid clusters found" << std::endl;
        out_x = std::nan("");
        out_y = std::nan("");
		return;
	}

	// Find largest cluster
	auto &largest_cluster = *std::max_element(clusters.begin(), clusters.end(), [](const auto &a, const auto &b) { return a.size() < b.size(); });

	if (largest_cluster.size() < MIN_CLUSTER_SIZE) {
		std::cout << "Insufficient cluster density" << std::endl;
        out_x = std::nan("");
        out_y = std::nan("");
		return;
	}

	// Calculate final position
	auto [final_x, final_y] = calculate_mean(largest_cluster);
	auto [final_std_x, final_std_y] = calculate_std_dev(largest_cluster, final_x, final_y);

	if (final_std_x > MAX_ACCEPTABLE_STD || final_std_y > MAX_ACCEPTABLE_STD) {
		std::cout << "Excessive variance in final position" << std::endl;
        out_x = std::nan("");
        out_y = std::nan("");
		return;
	}
    
    out_x = final_x;
    out_y = final_y;
}

std::pair<double, double> TrafficClient::calculate_mean(std::array<std::pair<double, double>, 32> &data) {
	double sum_x = 0.0, sum_y = 0.0;
	for (const auto &p : data) {
		sum_x += p.first;
		sum_y += p.second;
	}
	return {sum_x / data.size(), sum_y / data.size()};
}

std::pair<double, double> TrafficClient::calculate_std_dev(std::array<std::pair<double, double>, 32> &data, double mean_x, double mean_y) {
	double var_x = 0.0, var_y = 0.0;
	for (const auto &p : data) {
		var_x += std::pow(p.first - mean_x, 2);
		var_y += std::pow(p.second - mean_y, 2);
	}
	return {std::sqrt(var_x / data.size()), std::sqrt(var_y / data.size())};
}

std::pair<double, double> TrafficClient::calculate_mean(std::vector<std::pair<double, double>> &data) {
	double sum_x = 0.0, sum_y = 0.0;
	for (const auto &p : data) {
		sum_x += p.first;
		sum_y += p.second;
	}
	return {sum_x / data.size(), sum_y / data.size()};
}

std::pair<double, double> TrafficClient::calculate_std_dev(std::vector<std::pair<double, double>> &data, double mean_x, double mean_y) {
	double var_x = 0.0, var_y = 0.0;
	for (const auto &p : data) {
		var_x += std::pow(p.first - mean_x, 2);
		var_y += std::pow(p.second - mean_y, 2);
	}
	return {std::sqrt(var_x / data.size()), std::sqrt(var_y / data.size())};
}

std::vector<std::pair<double, double>> TrafficClient::filter_outliers(std::array<std::pair<double, double>, 32> &data, double mean_x, double mean_y, double std_x, double std_y, double sigma) {
	std::vector<std::pair<double, double>> result;
	for (const auto &p : data) {
		if (std::abs(p.first - mean_x) < sigma * std_x && std::abs(p.second - mean_y) < sigma * std_y) {
			result.push_back(p);
		}
	}
	return result;
}

std::vector<std::vector<std::pair<double, double>>> TrafficClient::cluster_points(const std::vector<std::pair<double, double>> &points, double radius) {

	std::vector<std::vector<std::pair<double, double>>> clusters;
	std::vector<bool> processed(points.size(), false);

	for (size_t i = 0; i < points.size(); ++i) {
		if (processed[i])
			continue;

		std::vector<std::pair<double, double>> cluster;
		std::vector<size_t> queue{i};
		processed[i] = true;

		while (!queue.empty()) {
			size_t idx = queue.back();
			queue.pop_back();
			cluster.push_back(points[idx]);

			for (size_t j = 0; j < points.size(); ++j) {
				if (!processed[j] && std::hypot(points[idx].first - points[j].first, points[idx].second - points[j].second) < radius) {
					processed[j] = true;
					queue.push_back(j);
				}
			}
		}

		clusters.push_back(cluster);
	}

	return clusters;
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
