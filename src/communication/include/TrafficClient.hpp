#pragma once

#include "std_msgs/Float32MultiArray.h"
#include <chrono>
#include <cstdint>
#include <netinet/in.h>
#include <sys/types.h>
#include <thread>

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
	void send_data();
	// Encode
	void send_car_data(const Float32MultiArray &road_object);

  private:
	// Fields
	const std::string car_id = "PotholeAvoidance";
    const std::chrono::high_resolution_clock::duration freq = std::chrono::milliseconds(250);
    const double frequency = 0.25;
    std::chrono::high_resolution_clock::time_point last_send = std::chrono::high_resolution_clock::now();
	const uint16_t tcp_port = 5000;
	std::string server_address = "127.0.0.1";
	const size_t buffer_size = 1024;
	bool alive = true;
	bool connected = false;
	sockaddr_in tcp_address;
	int tcp_socket;
	std::thread poll;
	// Utility Methods
	void create_tcp_socket();
    bool can_send();
	void poll_connection();
    void send_car_id();
    std::string create_vehicle_pos(double x, double y);
    std::string create_vehicle_rot(double yaw);
    std::string create_vehicle_speed(double speed);
    std::string create_encountered_obstacle(const std::string &type, double x, double y);
};
