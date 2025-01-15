#pragma once

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <cstdint>
#include <functional>
#include <netinet/in.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <thread>
#include <vector>

class TcpClient {
  public:
	// Constructors
	TcpClient(const char *server_ip, const uint16_t server_port, const size_t buffer_size);
	TcpClient(const size_t buffer_size, const char *client_type);
	TcpClient(TcpClient &&) = default;
	TcpClient(const TcpClient &) = delete;
	TcpClient &operator=(TcpClient &&) = delete;
	TcpClient &operator=(const TcpClient &) = delete;
	~TcpClient();
	// Methods
	void initialize();
	std::queue<std::string> &get_strings();
	void send_type(const std::string &str);
	void send_string(const std::string &str);
	void send_image_rgb(const sensor_msgs::Image &img);
	void send_image_depth(const sensor_msgs::Image &img);
	void send_road_object(const std_msgs::Float32MultiArray &array);
	void send_waypoint(const std_msgs::Float32MultiArray &array);
	void send_sign(const std_msgs::Float32MultiArray &array);
	void send_message(const std_msgs::String &msg);

  private:
	// Fields
	const char *client_type = nullptr;
	const size_t buffer_size;
	const size_t header_size = 5;
	const size_t message_size = 4;
	bool alive = true;
	bool canSend = false;
	sockaddr_in address;
	int client_socket;
	std::thread receive;
	std::map<uint8_t, std::function<void(TcpClient *, std::vector<uint8_t> &)>> data_actions;
	std::vector<uint8_t> data_types;
	std::queue<std::string> strings;
	// Methods
	void listen();
	void parse_string(std::vector<uint8_t> &data);
};
