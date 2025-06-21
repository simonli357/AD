#pragma once

#include "KeyDealer.hpp"
#include "utils/constants.h"
#include <any>
#include <chrono>
#include <cstdint>
#include <netinet/in.h>
#include <std_msgs/Float32MultiArray.h>
#include <sys/types.h>
#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>
#include <thread>
#include "Tracking.h"

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
	// Methods
	void initialize();
	// Encode
	void send_car_data();

  private:
	// Fields
	int car_id = 3;
	const milliseconds frequency = milliseconds(250);
	steady_clock::time_point last_send_time = steady_clock::now();
	const uint16_t tcp_port = 5000;
	std::string server_address = "192.168.50.2";
	const size_t buffer_size = 1024;
	bool alive = true;
	bool connected = false;
	sockaddr_in tcp_address;
	int tcp_socket;
	std::thread main;
	std::unique_ptr<tbb::task_group> tasks;
	std::unique_ptr<KeyDealer> keyDealer;
	// Utility Methods
	void create_tcp_socket();
	void listen();
	void send_data();
	void send_car_id();
	void subscribeToLocationData();
	bool can_send();
	// Encode
	std::string create_vehicle_pos(double x, double y);
	std::string create_vehicle_rot(double yaw);
	std::string create_vehicle_speed(double speed);
	std::string create_encountered_obstacle(int type, double x, double y);
};
