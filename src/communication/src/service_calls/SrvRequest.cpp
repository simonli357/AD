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
	std::memcpy(&x0, datatypes[1].data(), sizeof(double));

	double y0;
	std::memcpy(&y0, datatypes[2].data(), sizeof(double));

	double yaw0;
	std::memcpy(&yaw0, datatypes[3].data(), sizeof(double));

	double dest_x;
	std::memcpy(&dest_x, datatypes[4].data(), sizeof(double));

	double dest_y;
	std::memcpy(&dest_y, datatypes[5].data(), sizeof(double));

	return SrvRequest::GoToSrv{vrefName, x0, y0, yaw0, dest_x, dest_y};
}

SrvRequest::GoToCmdSrv SrvRequest::parse_go_to_cmd_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();

	double dest_x;
	std::memcpy(&dest_x, datatypes[0].data(), sizeof(double));

	double dest_y;
	std::memcpy(&dest_y, datatypes[1].data(), sizeof(double));

	return SrvRequest::GoToCmdSrv{dest_x, dest_y};
}

SrvRequest::SetStatesSrv SrvRequest::parse_set_states_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();

	double x;
	std::memcpy(&x, datatypes[0].data(), sizeof(double));

	double y;
	std::memcpy(&y, datatypes[1].data(), sizeof(double));

	return SrvRequest::SetStatesSrv{x, y};
}

SrvRequest::WaypointsSrv SrvRequest::parse_waypoints_srv() {
	std::vector<std::vector<uint8_t>> datatypes = split();

	std::string pathName(datatypes[0].begin(), datatypes[0].end());

	std::string vrefName(datatypes[1].begin(), datatypes[1].end());

	double x0;
	std::memcpy(&x0, datatypes[2].data(), sizeof(double));

	double y0;
	std::memcpy(&y0, datatypes[3].data(), sizeof(double));

	double yaw0;
	std::memcpy(&yaw0, datatypes[4].data(), sizeof(double));

	return SrvRequest::WaypointsSrv{pathName, vrefName, x0, y0, yaw0};
}
