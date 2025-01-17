#pragma once

#include "Decoder.hpp"
#include <cstdint>
#include <string>
#include <vector>

class SrvRequest : public Decoder {
  public:
	SrvRequest(std::vector<uint8_t> &bytes);
	SrvRequest(SrvRequest &&) = default;
	SrvRequest(const SrvRequest &) = default;
	SrvRequest &operator=(SrvRequest &&) = delete;
	SrvRequest &operator=(const SrvRequest &) = delete;
	~SrvRequest() = default;

	struct GoToSrv {
		std::string vrefName;
		double x0;
		double y0;
		double yaw0;
		double dest_x;
		double dest_y;
	};

	struct GoToCmdSrv {
		double dest_x;
		double dest_y;
	};

	struct SetStatesSrv {
		double x;
		double y;
	};

	struct WaypointsSrv {
		std::string pathName;
		std::string vrefName;
		double x0;
		double y0;
		double yaw0;
	};

	GoToSrv parse_go_to_srv();
	GoToCmdSrv parse_go_to_cmd_srv();
	SetStatesSrv parse_set_states_srv();
	WaypointsSrv parse_waypoints_srv();

  private:
	void double_from_bytes(double &value, std::vector<uint8_t> &bytes);
};
