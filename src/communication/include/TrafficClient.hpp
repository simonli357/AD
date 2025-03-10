#pragma once

#include "std_msgs/Float32MultiArray.h"
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
	void send_vehicle_pos(double x, double y);
	void send_vehicle_rot(double yaw);
	void send_vehicle_speed(double speed);
	void send_encountered_obstacle(const std::string &type, double x, double y);

  private:
	// Fields
	const std::string car_id = "PotholeAvoidance";
	const uint16_t tcp_port = 5000;
	std::string server_address = "127.0.0.1";
	const size_t buffer_size = 1024;
	bool alive = true;
	bool connected = false;
	sockaddr_in tcp_address;
	int tcp_socket;
	std::thread receive;
	std::thread poll;
	// Utility Methods
	void create_tcp_socket();
	void poll_connection();
	void listen();
};
