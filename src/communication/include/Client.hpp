#pragma once

#include <cstdint>
#include <netinet/in.h>
#include <sensor_msgs/Image.h>
#include <thread>
#include <vector>

class Client {
public:
  // Constructor
  Client(const char *receiver_ip, const uint16_t receiver_port);
  Client(Client &&) = default;
  Client(const Client &) = delete;
  Client &operator=(Client &&) = delete;
  Client &operator=(const Client &) = delete;
  ~Client();
  // Methods
  void initialize();
  void sendImage(const sensor_msgs::Image &img);

private:
  // Fields
  const char *ip;
  const uint16_t port;
  int client_socket;
  sockaddr_in address;
  std::thread receive;
  // Methods
  std::vector<uint8_t> serializeImage(const sensor_msgs::Image &img);
  void listen();
};
