#pragma once

#include "msg/TriggerMsg.hpp"
#include "service_calls/GoToCmdSrv.hpp"
#include "service_calls/GoToSrv.hpp"
#include "service_calls/SetStatesSrv.hpp"
#include "service_calls/WaypointsSrv.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "utils/Lane2.h"
#include <chrono>
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
	TcpClient(const size_t buffer_size, const std::string client_type, const std::string ip_address);
	TcpClient(TcpClient &&) = default;
	TcpClient(const TcpClient &) = delete;
	TcpClient &operator=(TcpClient &&) = delete;
	TcpClient &operator=(const TcpClient &) = delete;
	~TcpClient();
	// Methods
	void initialize();
	// Storage
	std::queue<std::string> &get_strings();
	std::queue<std::unique_ptr<GoToSrv>> &get_go_to_srv_msgs();
	std::queue<std::unique_ptr<GoToCmdSrv>> &get_go_to_cmd_srv_msgs();
	std::queue<std::unique_ptr<SetStatesSrv>> &get_set_states_srv_msgs();
	std::queue<std::unique_ptr<WaypointsSrv>> &get_waypoints_srv_msgs();
	std::queue<bool> &get_start_srv_msgs();
	std::queue<std::unique_ptr<TriggerMsg>> &get_trigger_msgs();
	// Encode
	void send_type(std::string &str);
	void send_string(std::string &str);
	void send_lane2(const utils::Lane2 &lane);
	void send_image_rgb(const Image &img);
	void send_image_depth(const Image &img);
	void send_road_object(Float32MultiArray &array);
	void send_waypoint(Float32MultiArray &array);
	void send_sign(Float32MultiArray &array);
	void send_message(String &msg);
	void send_go_to_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals);
	void send_go_to_cmd_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals, bool success);
	void send_set_states_srv(bool success);
	void send_waypoints_srv(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals);
	void send_start_srv(bool started);
	void send_trigger(std_srvs::Trigger &trigger);

  private:
	// Fields
	std::string server_address = "127.0.0.1";
	std::string client_type;
	const size_t buffer_size;
	const size_t header_size = 5;
	const size_t message_size = 4;
	const uint32_t MAX_DGRAM = 65507;
	const std::chrono::milliseconds UDP_THROTTLE{16};
	bool alive = true;
	bool connected = false;
	bool tcp_can_send = false;
	sockaddr_in tcp_address;
	sockaddr_in udp_address;
	int tcp_socket;
	int udp_socket;
	std::thread receive;
	std::thread poll;
	std::map<uint8_t, std::function<void(TcpClient *, std::vector<uint8_t> &)>> tcp_data_actions;
	std::vector<uint8_t> tcp_data_types;
	std::vector<uint8_t> udp_data_types;
	// Storage
	std::queue<std::string> strings;
	std::queue<std::unique_ptr<GoToSrv>> go_to_srv_msgs;
	std::queue<std::unique_ptr<GoToCmdSrv>> go_to_cmd_srv_msgs;
	std::queue<std::unique_ptr<SetStatesSrv>> set_states_srv_msgs;
	std::queue<std::unique_ptr<WaypointsSrv>> waypoints_srv_msgs;
	std::queue<bool> start_srv_msgs;
	std::queue<std::unique_ptr<TriggerMsg>> trigger_msgs;
	// Utility Methods
	void create_tcp_socket();
	void create_udp_socket();
	void set_tcp_data_types();
	void set_tcp_data_actions();
	void set_udp_data_types();
	void poll_connection();
	void listen();
	// Decode
	void parse_string(std::vector<uint8_t> &bytes);
	void parse_go_to_srv(std::vector<uint8_t> &bytes);
	void parse_go_to_cmd_srv(std::vector<uint8_t> &bytes);
	void parse_set_states_srv(std::vector<uint8_t> &bytes);
	void parse_waypoints_srv(std::vector<uint8_t> &bytes);
	void parse_start_srv(std::vector<uint8_t> &bytes);
	void parse_trigger_msg(std::vector<uint8_t> &bytes);
};
