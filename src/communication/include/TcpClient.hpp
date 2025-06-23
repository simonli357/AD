#pragma once

#include "msg/Lane2Msg.hpp"
#include "msg/ParamsMsg.hpp"
#include "msg/RunMsg.hpp"
#include "msg/SWLoadMsg.hpp"
#include "msg/TriggerMsg.hpp"
#include "msg/ImuMsg.hpp"
#include "service_calls/GoToCmdSrv.hpp"
#include "service_calls/GoToSrv.hpp"
#include "service_calls/SetStatesSrv.hpp"
#include "service_calls/WaypointsSrv.hpp"
#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/String.h"
#include "std_srvs/Trigger.h"
#include "utils/Lane2.h"
#include <cstdint>
#include <functional>
#include <geometry_msgs/Pose.h>
#include <map>
#include <memory>
#include <netinet/in.h>
#include <opencv2/core/mat.hpp>
#include <sensor_msgs/Image.h>
#include <string>
#include <sys/types.h>
#include <tbb/concurrent_queue.h>
#include <tbb/enumerable_thread_specific.h>
#include <tbb/task_group.h>
#include <thread>
#include <tuple>
#include <vector>

using sensor_msgs::Image;
using std_msgs::Float32MultiArray;
using std_msgs::String;

class TcpClient {
  public:
	// Constructors
	TcpClient(bool use_tcp, const std::string client_type, const std::string ip_address);
	TcpClient(TcpClient &&) = delete;
	TcpClient(const TcpClient &) = delete;
	TcpClient &operator=(TcpClient &&) = delete;
	TcpClient &operator=(const TcpClient &) = delete;
	~TcpClient();
	// Fields
	bool tcp_can_send = false;
	// Methods
	void run();
	void set_image_quality(int quality) { rgb_img_quality = quality; }
	// Encode
	void send_type(const std::string &str);
	void send_string(const std::string &str);
	void send_string(const std::string &str, uint8_t datatype);
	void send_lane2(const utils::Lane2 &lane);
	void send_image_rgb(const cv::Mat &img);
	void send_image_depth(const cv::Mat &img);
	void send_road_object(const Float32MultiArray &array);
	void send_waypoint(const Float32MultiArray &array);
	void send_sign(const std::vector<float> &data);
	void send_steer(float steer);
	void send_swload();
	void send_message(const String &msg);
	void send_trigger(const std_srvs::Trigger &trigger);
	void send_params(const std::vector<double> &state_refs, const std::vector<double> &attributes);
	void send_go_to_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals);
	void send_go_to_cmd_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals, bool success);
	void send_set_states_srv(bool success);
	void send_waypoints_srv(const Float32MultiArray &state_refs, const Float32MultiArray &input_refs, const Float32MultiArray &wp_attributes, const Float32MultiArray &wp_normals);
	void send_start_srv(bool started);
	void send_run(float v_ref, const std::string &path_name, float x_init, float y_init, float yaw_init);
	void send_model_states(const geometry_msgs::Pose &msg);
	void send_imu_calib(float sys_calib, float gyro_calib, float mag_calib, float accel_calib);

	// Callbacks
	void set_send_run_callback(std::function<void()> cb) { send_run_callback = cb; }
	void set_ack_callback(std::function<void()> cb) { ack_callback = cb; }
	void set_trigger_response_callback(std::function<void(const std_srvs::TriggerResponse &)> cb) { trigger_response_callback = cb; }
	void set_go_to_cmd_callback(std::function<void(const std::vector<std::tuple<float, float>> &)> cb) { go_to_cmd_callback = cb; }
	void set_set_states_callback(std::function<void(double, double)> cb) { set_states_callback = cb; }
	void set_start_callback(std::function<void(bool)> cb) { start_callback = cb; }
	void set_waypoints_callback(std::function<void(double, double, double)> cb) { waypoints_callback = cb; }
	void set_yaw_callback(std::function<void(int)> cb) { yaw_callback = cb; }

  private:
	// Fields
	const uint16_t tcp_port = 49153;
	const uint16_t udp_port = 49154;
	std::string server_address = "127.0.0.1";
	std::string client_type;
	const int buffer_size = 2097152;
	const uint32_t MAX_DGRAM = 65507;
	const size_t header_size = 5;
	const size_t message_size = 4;
	int rgb_img_quality = 30;
	int swload_counter = 0;
	bool alive = true;
	bool connected = false;
	sockaddr_in tcp_address;
	sockaddr_in udp_address;
	int tcp_socket;
	int udp_socket;
	std::thread main;
	std::unique_ptr<tbb::task_group> tasks;
	std::map<uint8_t, std::function<void(TcpClient *, std::vector<uint8_t> &)>> tcp_data_actions;
	std::vector<uint8_t> tcp_data_types;
	std::vector<uint8_t> udp_data_types;
	// Messages
	std::unique_ptr<SWLoadMsg> swload;
	std::unique_ptr<Lane2Msg> lane2_msg;
	std::unique_ptr<ParamsMsg> params_msg;
	std::unique_ptr<RunMsg> run_msg;
	std::unique_ptr<TriggerMsg> trigger_msg;
	std::unique_ptr<ImuMsg> imu_msg;
	// Service calls
	std::unique_ptr<GoToCmdSrv> goto_cmd_srv;
	std::unique_ptr<GoToSrv> goto_srv;
	std::unique_ptr<WaypointsSrv> waypoints_srv;
	std::unique_ptr<SetStatesSrv> set_states_srv;
	// UDP buffers
	tbb::enumerable_thread_specific<std::array<uint8_t, 65507>> udp_buffers{};
	tbb::enumerable_thread_specific<std::vector<uchar>> image_buffers{};
	// Utility Methods
	void create_tcp_socket();
	void create_udp_socket();
	void set_tcp_data_types();
	void set_tcp_data_actions();
	void set_udp_data_types();
	void poll_connection();
	void listen();
	template <typename Callable> void add_stream_task(Callable &&lambda);
	template <typename Callable> void add_dgram_task(Callable &&lambda);
	// Callbacks
	std::function<void(const std_srvs::TriggerResponse &)> trigger_response_callback;
	std::function<void(const std::vector<std::tuple<float, float>> &)> go_to_cmd_callback;
	std::function<void(double, double)> set_states_callback;
	std::function<void(bool)> start_callback;
	std::function<void(double, double, double)> waypoints_callback;
	std::function<void()> send_run_callback;
	std::function<void()> ack_callback;
	std::function<void(int)> yaw_callback;
	// Decode
	void parse_string(std::vector<uint8_t> &bytes);
	void parse_yaw(std::vector<uint8_t> &bytes);
	void parse_trigger_msg(std::vector<uint8_t> &bytes);
	void parse_go_to_cmd_srv(std::vector<uint8_t> &bytes);
	void parse_set_states_srv(std::vector<uint8_t> &bytes);
	void parse_waypoints_srv(std::vector<uint8_t> &bytes);
	void parse_start_srv(std::vector<uint8_t> &bytes);
};
