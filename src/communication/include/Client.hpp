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

class Client {
  public:
	// Constructor
	Client(const char *server_ip, const uint16_t server_port, const size_t buffer_size);
	Client(Client &&) = default;
	Client(const Client &) = delete;
	Client &operator=(Client &&) = delete;
	Client &operator=(const Client &) = delete;
	~Client();
	// Methods
	void initialize();
	std::queue<std::string> &get_strings();
	void send_string(const std::string &str);
	void send_image(const sensor_msgs::Image &img);
    void send_float32_multi_array(const std_msgs::Float32MultiArray &array);
    void send_message(const std_msgs::String &msg);

  private:
	// Fields
	const size_t buffer_size;
	const size_t header_size = 5;
	const size_t message_size = 4;
	bool alive = true;
	sockaddr_in address;
	int client_socket;
	std::thread receive;
	std::map<uint8_t, std::function<void(Client *, std::vector<uint8_t> &)>> data_actions;
	std::vector<uint8_t> data_types;
	std::queue<std::string> strings;
	// Methods
	void listen();
	void parse_string(std::vector<uint8_t> &data);
};
