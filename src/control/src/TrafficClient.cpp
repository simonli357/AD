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
#include <algorithm>   // std::nth_element
#include <cmath>       // std::fabs
#include <limits>

using json = nlohmann::json;

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
	main = std::thread(&TrafficClient::initialize, this);
	ThreadPools::communication.execute([this] { tasks = std::make_unique<tbb::task_group>(); });
	this->car_id = Tunable::gps_id;
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
	if (tcp_socket == -1) {
		std::cerr << "Failed to create TCP socket: " << strerror(errno) << std::endl;
		exit(EXIT_FAILURE);
	}
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
            std::this_thread::sleep_for(std::chrono::milliseconds(2500));
            continue;
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

bool TrafficClient::get_car_position(double& out_x, double& out_y)
{
    // ---------- constants ----------
		std::cout << "hi-1" << std::endl;
    static constexpr std::size_t BUF_CAP     = 32;   // compile-time!
    static constexpr std::size_t MIN_SAMPLES = 6;
    static constexpr double      MAD_FACTOR  = 3.0;  // ≈3σ for normal data

		std::cout << "hi" << std::endl;
    if (!enough_points) {                        // not filled a full lap yet
        out_x = out_y = 1000.0;;
        return false;
    }

		std::cout << "hi1" << std::endl;
    // ---------- snapshot under lock ----------
    std::array<std::pair<double,double>, BUF_CAP> snapshot;
    {
        std::shared_lock lk(pos_mtx);
        snapshot = car_positions;                // std::array *is* assignable
    }

    // ---------- split into X / Y ----------
    std::array<double, BUF_CAP> xs{}, ys{};
    for (std::size_t i = 0; i < BUF_CAP; ++i) {
        xs[i] = snapshot[i].first;
        ys[i] = snapshot[i].second;
    }

		std::cout << "hi2" << std::endl;
    const std::size_t mid = BUF_CAP / 2;
    std::nth_element(xs.begin(), xs.begin() + mid, xs.end());
    std::nth_element(ys.begin(), ys.begin() + mid, ys.end());
    const double med_x = xs[mid];
    const double med_y = ys[mid];

    // ---------- MAD ----------
    std::array<double, BUF_CAP> dxs{}, dys{};
    for (std::size_t i = 0; i < BUF_CAP; ++i) {
        dxs[i] = std::fabs(snapshot[i].first  - med_x);
        dys[i] = std::fabs(snapshot[i].second - med_y);
    }
    std::nth_element(dxs.begin(), dxs.begin() + mid, dxs.end());
    std::nth_element(dys.begin(), dys.begin() + mid, dys.end());
    const double mad_x = dxs[mid];
    const double mad_y = dys[mid];

    if (mad_x == 0.0 && mad_y == 0.0) {          // perfect overlap
        out_x = med_x;
        out_y = med_y;
        return true;
    }
		std::cout << "hi3" << std::endl;

    const double thr_x = MAD_FACTOR * 1.4826 * mad_x; // 1.4826 ≈ 1/Φ⁻¹(0.75)
    const double thr_y = MAD_FACTOR * 1.4826 * mad_y;

    // ---------- robust mean of inliers ----------
    double sum_x = 0.0, sum_y = 0.0;
    std::size_t count = 0;

    for (auto const& p : snapshot) {
        if (std::fabs(p.first  - med_x) <= thr_x &&
            std::fabs(p.second - med_y) <= thr_y)
        {
            sum_x += p.first;
            sum_y += p.second;
            ++count;
        }
    }
		std::cout << "hi4" << std::endl;

    if (count < MIN_SAMPLES) {                   // still too noisy
        out_x = out_y = std::numeric_limits<double>::quiet_NaN();
        return false;
    }

    out_x = sum_x / static_cast<double>(count);
    out_y = sum_y / static_cast<double>(count);
		std::cout << "hi5" << std::endl;
    return true;
}

std::pair<double, double> TrafficClient::calculate_mean(const std::array<std::pair<double, double>, 32> &data) {
	double sum_x = 0.0, sum_y = 0.0;
	for (const auto &p : data) {
		sum_x += p.first;
		sum_y += p.second;
	}
	return {sum_x / data.size(), sum_y / data.size()};
}

std::pair<double, double> TrafficClient::calculate_std_dev(const std::array<std::pair<double, double>, 32> &data, double mean_x, double mean_y) {
	double var_x = 0.0, var_y = 0.0;
	for (const auto &p : data) {
		var_x += std::pow(p.first - mean_x, 2);
		var_y += std::pow(p.second - mean_y, 2);
	}
	return {std::sqrt(var_x / data.size()), std::sqrt(var_y / data.size())};
}

std::pair<double, double> TrafficClient::calculate_mean(const std::vector<std::pair<double, double>> &data) {
	double sum_x = 0.0, sum_y = 0.0;
	for (const auto &p : data) {
		sum_x += p.first;
		sum_y += p.second;
	}
	return {sum_x / data.size(), sum_y / data.size()};
}

std::pair<double, double> TrafficClient::calculate_std_dev(const std::vector<std::pair<double, double>> &data, double mean_x, double mean_y) {
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
