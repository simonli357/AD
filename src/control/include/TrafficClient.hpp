#pragma once

#include "Tracking.h"
#include "Tunable.h"
#include <chrono>
#include <cstdint>
#include <netinet/in.h>
#include <string>
#include <std_msgs/Float32MultiArray.h>
#include <sys/types.h>
#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>
#include <thread>
#include <vector>

using namespace std::chrono;
using namespace VehicleConstants;
using std_msgs::Float32MultiArray;

class TrafficClient {
  public:
	// Constructors
	TrafficClient(const std::string ip_address);
	TrafficClient(TrafficClient &&) = delete;
	TrafficClient(const TrafficClient &) = delete;
	TrafficClient &operator=(TrafficClient &&) = delete;
	TrafficClient &operator=(const TrafficClient &) = delete;
	~TrafficClient();
	// Fields
	bool tcp_can_send = false;
	bool enough_points = false;
	// Methods
	void initialize();
	void clear_positions();
	std::pair<double, double> get_car_position();
	// Encode
	void send_car_data();

  private:
	// Fields
	int car_id = 6;
	const milliseconds frequency = milliseconds(250);
	const size_t num_points = Tunable::gps_points;
	std::vector<std::pair<double, double>> car_positions;
	size_t array_ptr = 0;
	steady_clock::time_point last_send_time = steady_clock::now();
	uint16_t tcp_port = 5000;
	const uint16_t udp_discovery_port = 9000;
	std::string server_address = "192.168.50.2";
	const size_t buffer_size = 1024;
	bool alive = true;
	bool connected = false;
	sockaddr_in tcp_address{};
	int tcp_socket = -1;
	std::string receive_buffer;
	std::thread main;
	std::unique_ptr<tbb::task_group> tasks;
	// Utility Methods
	bool discover_traffic_server(milliseconds timeout);
	void create_tcp_socket();
	bool connect_tcp_socket(milliseconds timeout);
	void listen();
	void send_data();
	void subscribeToLocationData();
	bool can_send();
	void handle_location_data(double x, double y, double z);
	std::pair<double, double> calculate_mean(std::vector<std::pair<double, double>> &data);
	std::pair<double, double> calculate_std_dev(std::vector<std::pair<double, double>> &data, double mean_x, double mean_y);
	std::vector<std::pair<double, double>> filter_outliers(std::vector<std::pair<double, double>> &data, double mean_x, double mean_y, double std_x, double std_y, double sigma);
	std::vector<std::vector<std::pair<double, double>>> cluster_points(const std::vector<std::pair<double, double>> &points, double radius);
	// Encode
	std::string create_vehicle_pos(double x, double y);
	std::string create_vehicle_rot(double yaw);
	std::string create_vehicle_speed(double speed);
	std::string create_encountered_obstacle(int type, double x, double y);
};
;
