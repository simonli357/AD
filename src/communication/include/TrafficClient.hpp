#pragma once

#include "std_msgs/Float32MultiArray.h"
#include <any>
#include <chrono>
#include <cstdint>
#include <netinet/in.h>
#include <sys/types.h>
#include <tbb/concurrent_queue.h>
#include <tbb/task_group.h>
#include <thread>

using namespace std::chrono;
using std_msgs::Float32MultiArray;

class TrafficClient {
  public:
	// Constructors
	TrafficClient(const std::string ip_address);
	TrafficClient(TrafficClient &&) = default;
	TrafficClient(const TrafficClient &) = delete;
	TrafficClient &operator=(TrafficClient &&) = delete;
	TrafficClient &operator=(const TrafficClient &) = delete;
	~TrafficClient();
	// Fields
	bool tcp_can_send = false;
	// Methods
	void initialize();
	// Encode
	void send_car_data(const Float32MultiArray &road_object);

  private:
	// Fields
	int car_id = 3;
	const milliseconds frequency = milliseconds(250);
	steady_clock::time_point last_send_time = steady_clock::now();
	const uint16_t tcp_port = 5000;
	std::string server_address = "127.0.0.1";
	const size_t buffer_size = 1024;
	bool alive = true;
	bool connected = false;
	sockaddr_in tcp_address;
	int tcp_socket;
	std::thread main;
	std::unique_ptr<tbb::task_group> tasks;
	// Task Queue
	tbb::concurrent_queue<std::any> stream_tasks;
	// Utility Methods
	void create_tcp_socket();
	void poll_connection();
	void send_data();
	void send_car_id();
	template <typename Callable> void add_stream_task(Callable &&lambda);
	// Encode
	std::string create_vehicle_pos(double x, double y);
	std::string create_vehicle_rot(double yaw);
	std::string create_vehicle_speed(double speed);
	std::string create_encountered_obstacle(int type, double x, double y);
};
