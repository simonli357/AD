#pragma once

#include <cstdint>
#include <functional>
#include <netinet/in.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include <vector>

class Client {
public:
  // Constructor
  Client(const char *server_ip, const uint16_t server_port);
  Client(Client &&) = default;
  Client(const Client &) = delete;
  Client &operator=(Client &&) = delete;
  Client &operator=(const Client &) = delete;
  ~Client();
  // Methods
  void initialize();
  void send_string(const std::string& str);

private:
  // Fields
  const char *ip;
  const uint16_t port;
  int client_socket;
  sockaddr_in address;
  std::thread receive;
  bool alive = true;
  std::map<uint8_t, std::function<void(Client*, std::vector<uint8_t>&)>> data_actions;
  std::vector<uint8_t> data_types;
  // Methods
  void listen();
  void parse_string(std::vector<uint8_t>& data);
  // std::vector<uint8_t> serialize_image(const sensor_msgs::Image &img);
};
