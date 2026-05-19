#include "TrafficClient.hpp"
#include "utils/constants.h"
#include <algorithm>
#include <arpa/inet.h>
#include <array>
#include <cerrno>
#include <chrono>
#include <cstring>
#include <cv_bridge/cv_bridge.h>
#include <exception>
#include <fcntl.h>
#include <cmath>
#include <iostream>
#include <netinet/in.h>
#include <nlohmann/json.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/opencv.hpp>
#include <optional>
#include <ostream>
#include <poll.h>
#include <ros/ros.h>
#include <string>
#include <sys/socket.h>
#include <tuple>
#include <unistd.h>
#include <vector>

using json = nlohmann::json;

namespace {
constexpr size_t max_receive_buffer_size = 64 * 1024;
constexpr double millimeter_coordinate_threshold = 100.0;

std::vector<std::string> extract_json_objects(std::string &buffer) {
	std::vector<std::string> objects;
	size_t object_start = std::string::npos;
	int depth = 0;
	bool in_string = false;
	bool escaped = false;

	for (size_t i = 0; i < buffer.size(); ++i) {
		const char c = buffer[i];

		if (in_string) {
			if (escaped) {
				escaped = false;
			} else if (c == '\\') {
				escaped = true;
			} else if (c == '"') {
				in_string = false;
			}
			continue;
		}

		if (c == '"') {
			in_string = true;
			continue;
		}

		if (c == '{') {
			if (depth == 0) {
				object_start = i;
			}
			++depth;
		} else if (c == '}' && depth > 0) {
			--depth;
			if (depth == 0 && object_start != std::string::npos) {
				objects.emplace_back(buffer.substr(object_start, i - object_start + 1));
				object_start = std::string::npos;
			}
		}
	}

	if (depth > 0 && object_start != std::string::npos) {
		buffer.erase(0, object_start);
	} else {
		buffer.clear();
	}

	if (buffer.size() > max_receive_buffer_size) {
		std::cerr << "TrafficClient: dropping oversized partial TCP JSON buffer" << std::endl;
		buffer.clear();
	}

	return objects;
}

double location_unit_scale(double x, double y, double z) {
	const double max_abs_coordinate = std::max({std::abs(x), std::abs(y), std::abs(z)});
	return max_abs_coordinate > millimeter_coordinate_threshold ? 0.001 : 1.0;
}

std::optional<double> read_json_double(const nlohmann::json &msg, const char *key) {
	if (!msg.contains(key) || msg.at(key).is_null()) {
		return std::nullopt;
	}

	const auto &value = msg.at(key);
	if (value.is_number()) {
		return value.get<double>();
	}

	if (value.is_string()) {
		try {
			const std::string raw = value.get<std::string>();
			size_t parsed_length = 0;
			const double parsed = std::stod(raw, &parsed_length);
			if (parsed_length == raw.size()) {
				return parsed;
			}
		} catch (const std::exception &) {
			return std::nullopt;
		}
	}

	return std::nullopt;
}

bool set_nonblocking(int socket_fd) {
	const int flags = fcntl(socket_fd, F_GETFL, 0);
	if (flags == -1) {
		return false;
	}
	return fcntl(socket_fd, F_SETFL, flags | O_NONBLOCK) != -1;
}

std::optional<uint16_t> parse_discovery_port(const std::string &payload) {
	const size_t colon = payload.rfind(':');
	if (colon == std::string::npos || colon + 1 >= payload.size()) {
		return std::nullopt;
	}

	try {
		const int parsed_port = std::stoi(payload.substr(colon + 1));
		if (parsed_port > 0 && parsed_port <= 65535) {
			return static_cast<uint16_t>(parsed_port);
		}
	} catch (const std::exception &) {
		return std::nullopt;
	}

	return std::nullopt;
}
} // namespace

TrafficClient::TrafficClient(const std::string ip_address) : server_address(ip_address) {
	tasks = std::make_unique<tbb::task_group>();
	this->car_id = Tunable::gps_id;
	this->car_positions.reserve(num_points);
	this->car_positions.resize(num_points);
	main = std::thread(&TrafficClient::initialize, this);
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

bool TrafficClient::discover_traffic_server(milliseconds timeout) {
	const int udp_socket = socket(AF_INET, SOCK_DGRAM, 0);
	if (udp_socket == -1) {
		std::cerr << "TrafficClient: failed to create UDP discovery socket: " << std::strerror(errno) << std::endl;
		return false;
	}

	const int reuse = 1;
	setsockopt(udp_socket, SOL_SOCKET, SO_REUSEADDR, &reuse, sizeof(reuse));

	sockaddr_in listen_address{};
	listen_address.sin_family = AF_INET;
	listen_address.sin_addr.s_addr = htonl(INADDR_ANY);
	listen_address.sin_port = htons(udp_discovery_port);

	if (bind(udp_socket, (struct sockaddr *)&listen_address, sizeof(listen_address)) != 0) {
		std::cerr << "TrafficClient: failed to bind UDP discovery port " << udp_discovery_port << ": " << std::strerror(errno) << std::endl;
		close(udp_socket);
		return false;
	}

	pollfd socket_poll{};
	socket_poll.fd = udp_socket;
	socket_poll.events = POLLIN;

	const int poll_result = poll(&socket_poll, 1, static_cast<int>(timeout.count()));
	if (poll_result <= 0) {
		close(udp_socket);
		return false;
	}

	std::array<char, 2048> buffer{};
	sockaddr_in sender_address{};
	socklen_t sender_length = sizeof(sender_address);
	const ssize_t bytes = recvfrom(udp_socket, buffer.data(), buffer.size(), 0, (struct sockaddr *)&sender_address, &sender_length);
	close(udp_socket);

	if (bytes <= 0) {
		return false;
	}

	const std::string datagram(buffer.data(), bytes);
	const std::string separator = "(-.-)";
	const size_t separator_pos = datagram.find(separator);
	if (separator_pos == std::string::npos) {
		std::cerr << "TrafficClient: UDP discovery datagram missing signature separator" << std::endl;
		return false;
	}

	const std::string payload = datagram.substr(separator_pos + separator.size());
	const auto discovered_port = parse_discovery_port(payload);
	if (!discovered_port) {
		std::cerr << "TrafficClient: UDP discovery payload did not contain a valid port: " << payload << std::endl;
		return false;
	}

	std::array<char, INET_ADDRSTRLEN> discovered_ip{};
	if (!inet_ntop(AF_INET, &sender_address.sin_addr, discovered_ip.data(), discovered_ip.size())) {
		return false;
	}

	server_address = discovered_ip.data();
	tcp_port = *discovered_port;
	std::cout << "TrafficClient: discovered Traffic Server at " << server_address << ":" << tcp_port << std::endl;
	return true;
}

void TrafficClient::create_tcp_socket() {
	if (tcp_socket != -1) {
		close(tcp_socket);
		tcp_socket = -1;
	}
	tcp_socket = socket(AF_INET, SOCK_STREAM, 0);
	if (tcp_socket == -1) {
		std::cerr << "TrafficClient: failed to create TCP socket: " << std::strerror(errno) << std::endl;
		return;
	}
	tcp_address.sin_family = AF_INET;
	tcp_address.sin_port = htons(tcp_port);
	if (inet_pton(AF_INET, server_address.c_str(), &tcp_address.sin_addr) != 1) {
		std::cerr << "TrafficClient: invalid Traffic Server IP address: " << server_address << std::endl;
		close(tcp_socket);
		tcp_socket = -1;
	}
}

bool TrafficClient::connect_tcp_socket(milliseconds timeout) {
	if (tcp_socket == -1) {
		return false;
	}

	if (!set_nonblocking(tcp_socket)) {
		std::cerr << "TrafficClient: failed to set TCP socket non-blocking: " << std::strerror(errno) << std::endl;
		return false;
	}

	const int rc = connect(tcp_socket, (struct sockaddr *)&tcp_address, sizeof(tcp_address));
	if (rc == 0 || errno == EISCONN) {
		return true;
	}

	if (errno != EINPROGRESS && errno != EALREADY) {
		std::cerr << "TrafficClient: connect failed: " << std::strerror(errno) << std::endl;
		return false;
	}

	pollfd socket_poll{};
	socket_poll.fd = tcp_socket;
	socket_poll.events = POLLOUT;

	const int poll_result = poll(&socket_poll, 1, static_cast<int>(timeout.count()));
	if (poll_result == 0) {
		std::cerr << "TrafficClient: connect timed out" << std::endl;
		return false;
	}
	if (poll_result < 0) {
		std::cerr << "TrafficClient: connect poll failed: " << std::strerror(errno) << std::endl;
		return false;
	}

	int socket_error = 0;
	socklen_t socket_error_len = sizeof(socket_error);
	if (getsockopt(tcp_socket, SOL_SOCKET, SO_ERROR, &socket_error, &socket_error_len) != 0) {
		std::cerr << "TrafficClient: failed to read connect status: " << std::strerror(errno) << std::endl;
		return false;
	}
	if (socket_error != 0) {
		std::cerr << "TrafficClient: connect failed: " << std::strerror(socket_error) << std::endl;
		return false;
	}

	return true;
}

void TrafficClient::initialize() {
	while (alive) {
		discover_traffic_server(std::chrono::milliseconds(1200));
		create_tcp_socket();
		std::cout << "Connecting to Traffic Server at " << server_address << ":" << tcp_port << "\n" << std::endl;

		if (!connect_tcp_socket(std::chrono::milliseconds(1000))) {
			if (tcp_socket != -1) {
				close(tcp_socket);
				tcp_socket = -1;
			}
			std::this_thread::sleep_for(std::chrono::milliseconds(500));
			continue;
		}

		std::cout << "Successfully connected to Traffic Server \n" << std::endl;
		connected = true;
		subscribeToLocationData();
		std::this_thread::sleep_for(std::chrono::milliseconds(3000));
		tcp_can_send = true;
		listen();
		if (tcp_socket != -1) {
			close(tcp_socket);
			tcp_socket = -1;
		}
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
			receive_buffer.append(reinterpret_cast<char *>(buffer.data()), bytes);

			for (const auto &payload : extract_json_objects(receive_buffer)) {
				nlohmann::json msg = nlohmann::json::parse(payload, nullptr, false);
				if (msg.is_discarded()) {
					std::cerr << "TrafficClient: received invalid JSON from Traffic Server: " << payload << std::endl;
					continue;
				}

				if (msg.contains("error")) {
					std::cerr << "TrafficClient: Traffic Server rejected message: " << msg.dump() << std::endl;
					continue;
				}

				const std::string type = msg.value("type", "");
				if (!type.empty() && type != "location") {
					std::cout << "TrafficClient: non-location message from Traffic Server: " << msg.dump() << std::endl;
					continue;
				}

				const auto x = read_json_double(msg, "x");
				const auto y = read_json_double(msg, "y");
				const auto z = read_json_double(msg, "z").value_or(0.0);

				if (!x || !y) {
					std::cerr << "TrafficClient: location message missing x/y: " << msg.dump() << std::endl;
					continue;
				}

				if (!std::isfinite(*x) || !std::isfinite(*y) || !std::isfinite(z)) {
					std::cerr << "TrafficClient: non-finite location payload: " << msg.dump() << std::endl;
					continue;
				}

				const double scale = location_unit_scale(*x, *y, z);
				if (scale != 1.0) {
					static bool logged_mm_scale = false;
					if (!logged_mm_scale) {
						std::cout << "TrafficClient: detected millimetre-scale location payload; converting to metres" << std::endl;
						logged_mm_scale = true;
					}
				}

				std::cout << "GPS DATA: " << msg.dump() << std::endl;

				handle_location_data(*x * scale, *y * scale, z * scale);
			}
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
	car_positions.clear();
	array_ptr = 0;
}

std::pair<double, double> TrafficClient::get_car_position() {
	constexpr double MAX_ACCEPTABLE_STD = 0.25;
	constexpr double CLUSTER_RADIUS = 0.3;
	constexpr size_t MIN_CLUSTER_SIZE = 6;

	if (!enough_points) {
		std::this_thread::sleep_for(std::chrono::milliseconds(100));
		return {0.0, 0.0};
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
		return {0.0, 0.0};
	}

	// Find largest cluster
	auto &largest_cluster = *std::max_element(clusters.begin(), clusters.end(), [](const auto &a, const auto &b) { return a.size() < b.size(); });

	if (largest_cluster.size() < MIN_CLUSTER_SIZE) {
		std::cout << "Insufficient cluster density" << std::endl;
		return {0.0, 0.0};
	}

	// Calculate final position
	auto [final_x, final_y] = calculate_mean(largest_cluster);
	auto [final_std_x, final_std_y] = calculate_std_dev(largest_cluster, final_x, final_y);

	if (final_std_x > MAX_ACCEPTABLE_STD || final_std_y > MAX_ACCEPTABLE_STD) {
		std::cout << "Excessive variance in final position" << std::endl;
		return {0.0, 0.0};
	}

	return {final_x, final_y};
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

std::vector<std::pair<double, double>> TrafficClient::filter_outliers(std::vector<std::pair<double, double>> &data, double mean_x, double mean_y, double std_x, double std_y, double sigma) {
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
		std::shared_ptr<Tracking::EgoCarObject> ego_car;
		std::vector<std::shared_ptr<Tracking::RoadObject>> road_objects;
		std::vector<std::shared_ptr<Tracking::KnownStaticObject>> road_known_static_objects;
		std::vector<std::shared_ptr<Tracking::DynamicObject>> road_cars;
		std::vector<std::shared_ptr<Tracking::DynamicObject>> road_pedestrians;
		{
			std::lock_guard<std::mutex> lock(Tracking::container_mutex);
			ego_car = Tracking::ego_car;
			road_objects = Tracking::road_objects;
			road_known_static_objects = Tracking::road_known_static_objects;
			road_cars = Tracking::road_cars;
			road_pedestrians = Tracking::road_pedestrians;
		}

		if (!ego_car) {
			return;
		}

		double ego_x = 0.0;
		double ego_y = 0.0;
		double ego_yaw = 0.0;
		double ego_speed = 0.0;
		{
			std::lock_guard<std::mutex> lock(ego_car->mtx);
			ego_x = ego_car->x;
			ego_y = ego_car->y;
			ego_yaw = ego_car->yaw;
			ego_speed = ego_car->speed;
		}

		std::string v_pos = create_vehicle_pos(ego_x, ego_y);
		std::string v_rot = create_vehicle_rot(ego_yaw);
		std::string v_speed = create_vehicle_speed(ego_speed);
		std::string msg_string = v_pos + v_rot + v_speed;
		std::vector<int> newly_reported_history_ids;

		auto append_history_object_once = [this, &msg_string, &newly_reported_history_ids](const std::shared_ptr<Tracking::RoadObject> &obj, int history_id) {
			if (!obj || history_id <= 0) {
				return;
			}

			int tracking_id = -1;
			int type_index = -1;
			double x = 0.0;
			double y = 0.0;
			double cumulative_confidence = 0.0;
			{
				std::lock_guard<std::mutex> lock(obj->mtx);
				tracking_id = obj->id;
				type_index = static_cast<int>(obj->type);
				x = obj->x;
				y = obj->y;
				cumulative_confidence = obj->cumulative_confidence;
			}

			if (type_index < 0 || type_index >= static_cast<int>(Tunable::cumulative_confidence_thresholds.size())) {
				return;
			}
			if (cumulative_confidence < Tunable::cumulative_confidence_thresholds[type_index]) {
				return;
			}

			{
				std::lock_guard<std::mutex> lock(reported_history_mutex);
				if (!reported_history_object_ids.insert(tracking_id).second) {
					return;
				}
				newly_reported_history_ids.push_back(tracking_id);
			}

			msg_string += create_encountered_obstacle(history_id, x, y);
		};

		auto get_object_type = [](const std::shared_ptr<Tracking::RoadObject> &obj, OBJECT &type) {
			if (!obj) {
				return false;
			}
			std::lock_guard<std::mutex> lock(obj->mtx);
			type = obj->type;
			return true;
		};

		for (auto &obj : road_objects) {
			OBJECT type;
			if (!get_object_type(obj, type)) {
				continue;
			}
			int id = -1;
			if (type == OBJECT::ONEWAY) {
				id = 8;
			} else if (type == OBJECT::NOENTRY) {
				id = 9;
			} else if (type == OBJECT::RAMP) {
				id = 17;
			} else if (type == OBJECT::TUNNEL) {
				id = 16;
			} else if (type == OBJECT::FOG) {
				id = 15;
			}
			append_history_object_once(obj, id);
		}
		for (auto &obj : road_known_static_objects) {
			OBJECT type;
			if (!get_object_type(obj, type)) {
				continue;
			}
			int id = -1;
			if (type == OBJECT::LIGHTS || type == OBJECT::GREENLIGHT || type == OBJECT::YELLOWLIGHT || type == OBJECT::REDLIGHT) {
				auto light_obj = std::dynamic_pointer_cast<Tracking::LightObject>(obj);
				if (!light_obj) {
					continue;
				}
				id = 14;
			} else if (type == OBJECT::HIGHWAYENTRANCE) {
				id = 5;
			} else if (type == OBJECT::STOPSIGN) {
				id = 1;
			} else if (type == OBJECT::ROUNDABOUT) {
				id = 7;
			} else if (type == OBJECT::PARK) {
				id = 3;
			} else if (type == OBJECT::CROSSWALK) {
				id = 4;
			} else if (type == OBJECT::HIGHWAYEXIT) {
				id = 6;
			} else if (type == OBJECT::PRIORITY) {
				id = 2;
			}
			append_history_object_once(obj, id);
		}

		for (auto &car : road_cars) {
			int id = -1;
			if (!car) {
				continue;
			}
			bool parked = false;
			{
				std::lock_guard<std::mutex> lock(car->mtx);
				parked = car->parked;
			}
			if (parked) {
				id = 10;
			}
			append_history_object_once(car, id);
		}

		for (auto &car : road_pedestrians) {
			int id = 11;
			auto pedestrian_obj = std::dynamic_pointer_cast<Tracking::PedestrianObject>(car);
			if (pedestrian_obj) {
				bool on_crosswalk = false;
				{
					std::lock_guard<std::mutex> lock(pedestrian_obj->mtx);
					on_crosswalk = pedestrian_obj->on_crosswalk;
				}
				if (on_crosswalk) {
					id = 12;
				}
			}
			append_history_object_once(car, id);
		}

		const ssize_t bytes_sent = send(tcp_socket, msg_string.data(), msg_string.size(), MSG_NOSIGNAL);
		if (bytes_sent != static_cast<ssize_t>(msg_string.size()) && !newly_reported_history_ids.empty()) {
			std::lock_guard<std::mutex> lock(reported_history_mutex);
			for (int id : newly_reported_history_ids) {
				reported_history_object_ids.erase(id);
			}
		}
	});
}
