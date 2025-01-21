#include "service_calls/SrvRequest.hpp"
#include <boost/lexical_cast.hpp>
#include <cstdint>
#include <cstring>
#include <iostream>
#include <string>
#include <vector>

SrvRequest::SrvRequest(std::vector<uint8_t> &bytes) : Decoder(bytes) {}

SrvRequest::GoToSrv SrvRequest::parse_go_to_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	std::string vrefName(datatypes[0].begin(), datatypes[0].end());
	double x0 = double_from_bytes(datatypes[1]);
	double y0 = double_from_bytes(datatypes[2]);
	double yaw0 = double_from_bytes(datatypes[3]);
	double dest_x = double_from_bytes(datatypes[4]);
	double dest_y = double_from_bytes(datatypes[5]);
	return SrvRequest::GoToSrv{vrefName, x0, y0, yaw0, dest_x, dest_y};
}

SrvRequest::GoToCmdSrv SrvRequest::parse_go_to_cmd_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	double dest_x = double_from_bytes(datatypes[0]);
	double dest_y = double_from_bytes(datatypes[1]);
	return SrvRequest::GoToCmdSrv{dest_x, dest_y};
}

SrvRequest::SetStatesSrv SrvRequest::parse_set_states_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	double x = double_from_bytes(datatypes[0]);
	double y = double_from_bytes(datatypes[1]);
	return SrvRequest::SetStatesSrv{x, y};
}

SrvRequest::WaypointsSrv SrvRequest::parse_waypoints_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();
	std::string pathName(datatypes[0].begin(), datatypes[0].end());
	std::string vrefName(datatypes[1].begin(), datatypes[1].end());
	double x0 = double_from_bytes(datatypes[2]);
	double y0 = double_from_bytes(datatypes[3]);
	double yaw0 = double_from_bytes(datatypes[4]);
	return SrvRequest::WaypointsSrv{pathName, vrefName, x0, y0, yaw0};
}

double SrvRequest::double_from_bytes(std::vector<uint8_t> &bytes) {
	std::string double_str(bytes.begin(), bytes.end());
    double d;
    std::istringstream(double_str) >> d;
    return d;
}
