#include "msg/RunMsg.hpp"
#include <cstdint>

void RunMsg::encode(float v_ref, const std::string &path_name, float x_init, float y_init, float yaw_init) {
    this->v_ref = v_ref;
    this->path_name = path_name;
    this->x_init = x_init;
    this->y_init = y_init;
    this->yaw_init = yaw_init;
    v_ref_length = sizeof(v_ref);
    path_name_length = path_name.size();
    x_init_length = sizeof(x_init);
    y_init_length = sizeof(y_init);
    yaw_init_length = sizeof(yaw_init);
	data_length = v_ref_length + path_name_length + x_init_length + y_init_length + yaw_init_length;
}

uint32_t RunMsg::compute_lengths_length() { return lengths_length; }

uint32_t RunMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> RunMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &v_ref_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &path_name_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &x_init_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 4, &y_init_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 5, &yaw_init_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> RunMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	size_t offset = 0;
	std::memcpy(data.data(), &v_ref, v_ref_length);
	offset += v_ref_length;

	std::memcpy(data.data() + offset, path_name.data(), path_name_length);
    offset += path_name_length;

    std::memcpy(data.data() + offset, &x_init, x_init_length);
    offset += x_init_length;

    std::memcpy(data.data() + offset, &y_init, y_init_length);
    offset += y_init_length;

    std::memcpy(data.data() + offset, &yaw_init, yaw_init_length);

	return data;
}
