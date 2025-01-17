#include "service_calls/SrvRequest.hpp"
#include <cstdint>
#include <cstring>
#include <string>
#include <vector>

SrvRequest::SrvRequest(std::vector<uint8_t> &bytes) : Decoder(bytes) {}

SrvRequest::GoToSrv SrvRequest::parse_go_to_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	std::string vrefName(datatypes[0].begin(), datatypes[0].end());
	double x0;
    double_from_bytes(x0, datatypes[1]);
	double y0;
    double_from_bytes(y0, datatypes[2]);
	double yaw0;
    double_from_bytes(yaw0, datatypes[3]);
	double dest_x;
    double_from_bytes(dest_x, datatypes[4]);
	double dest_y;
    double_from_bytes(dest_y, datatypes[5]);
	return SrvRequest::GoToSrv{vrefName, x0, y0, yaw0, dest_x, dest_y};
}

SrvRequest::GoToCmdSrv SrvRequest::parse_go_to_cmd_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	double dest_x;
    double_from_bytes(dest_x, datatypes[0]);
	double dest_y;
    double_from_bytes(dest_y, datatypes[1]);
	return SrvRequest::GoToCmdSrv{dest_x, dest_y};
}

SrvRequest::SetStatesSrv SrvRequest::parse_set_states_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	double x;
    double_from_bytes(x, datatypes[0]);
	double y;
    double_from_bytes(y, datatypes[1]);
	return SrvRequest::SetStatesSrv{x, y};
}

SrvRequest::WaypointsSrv SrvRequest::parse_waypoints_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	std::string pathName(datatypes[0].begin(), datatypes[0].end());
	std::string vrefName(datatypes[1].begin(), datatypes[1].end());
	double x0;
    double_from_bytes(x0, datatypes[2]);
	double y0;
    double_from_bytes(y0, datatypes[3]);
	double yaw0;
    double_from_bytes(yaw0, datatypes[4]);
	return SrvRequest::WaypointsSrv{pathName, vrefName, x0, y0, yaw0};
}

void SrvRequest::double_from_bytes(double &value, std::vector<uint8_t> &bytes) {
    uint8_t reversed_bytes[sizeof(double)];
    for (size_t i = 0; i < sizeof(double); ++i) {
        reversed_bytes[i] = bytes[sizeof(double) - 1 - i];
    }
    std::memcpy(&value, reversed_bytes, sizeof(double));
}
