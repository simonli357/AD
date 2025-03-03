#include "msg/ObjectMsg.hpp"
#include <cstdint>

ObjectMsg::ObjectMsg() {}

ObjectMsg::ObjectMsg(std::string &type, float x, float y) : type(type), x(x), y(y) {
    type_length = type.size();
    x_length = sizeof(x);
    y_length = sizeof(y);
	data_length = type_length + x_length + y_length;
}

uint32_t ObjectMsg::compute_lengths_length() { return lengths_length; }

uint32_t ObjectMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> ObjectMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &type_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &x_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &y_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> ObjectMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	size_t offset = 0;
	std::memcpy(data.data(), type.data(), type_length);
    offset += type_length;

    std::memcpy(data.data() + offset, &x, x_length);
    offset += x_length;

    std::memcpy(data.data() + offset, &y, y_length);

	return data;
}
