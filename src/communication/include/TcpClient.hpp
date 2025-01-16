#pragma once

#include "service_calls/SrvRequest.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include <cstdint>
#include <functional>
#include <netinet/in.h>
#include <queue>
#include <sensor_msgs/Image.h>
#include <sys/types.h>
#include <thread>
#include <vector>

using sensor_msgs::Image;
using std_msgs::Float32MultiArray;
using std_msgs::String;

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
	// Storage
	std::queue<std::string> &get_strings();
	std::queue<SrvRequest::GoToSrv> &get_go_to_srv_msgs();
	std::queue<SrvRequest::GoToCmdSrv> &get_go_to_cmd_srv_msgs();
	std::queue<SrvRequest::SetStatesSrv> &get_set_states_srv_msgs();
	std::queue<SrvRequest::WaypointsSrv> &get_waypoints_srv_msgs();
	// Encode
	void send_type(const std::string &str);
	void send_string(const std::string &str);
	void send_image_rgb(const Image &img);
	void send_image_depth(const Image &img);
	void send_road_object(const Float32MultiArray &array);
	void send_waypoint(const Float32MultiArray &array);
	void send_sign(const Float32MultiArray &array);
	void send_message(const String &msg);
	void send_go_to_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals);
	void send_go_to_cmd_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals, bool success);
	void send_set_states_srv(bool success);
	void send_waypoints_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals);

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
	// Storage
	std::queue<std::string> strings;
	std::queue<SrvRequest::GoToSrv> go_to_srv_msgs;
	std::queue<SrvRequest::GoToCmdSrv> go_to_cmd_srv_msgs;
	std::queue<SrvRequest::SetStatesSrv> set_states_srv_msgs;
	std::queue<SrvRequest::WaypointsSrv> waypoints_srv_msgs;
	// Methods
	void set_data_types();
	void set_data_actions();
	void listen();
	// Decode
	void parse_string(std::vector<uint8_t> &bytes);
	void parse_go_to_srv(std::vector<uint8_t> &bytes);
	void parse_go_to_cmd_srv(std::vector<uint8_t> &bytes);
	void parse_set_states_srv(std::vector<uint8_t> &bytes);
	void parse_waypoints_srv(std::vector<uint8_t> &bytes);
};
