#include "SWLoadMsg.hpp"
#include "ros/serialization.h"

SWLoadMsg::SWLoadMsg(std_msgs::Float64MultiArray &cores_usage, float cpu_usage, float ram_usage, float temperature, float heap_usage, float stack_usage) 
    : cores_usage(cores_usage), cpu_usage(cpu_usage), ram_usage(ram_usage), temperature(temperature), heap_usage(heap_usage), stack_usage(stack_usage) {
    cores_usage_length = ros::serialization::serializationLength(cores_usage);
    cpu_usage_length = sizeof(cpu_usage);
    ram_usage_length = sizeof(ram_usage);
    temperature_length = sizeof(temperature);
    heap_usage_length = sizeof(heap_usage);
    stack_usage_length = sizeof(stack_usage);
	data_length = cores_usage_length + cpu_usage_length + ram_usage_length + temperature_length + heap_usage_length + stack_usage_length;
}

uint32_t SWLoadMsg::compute_lengths_length() { return lengths_length; }

uint32_t SWLoadMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> SWLoadMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &cores_usage_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &cpu_usage_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &ram_usage_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 4, &temperature_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 5, &heap_usage_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 6, &stack_usage_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> SWLoadMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> cores_usage_data = serializeFloat64MultiArray(cores_usage);

	size_t offset = 0;
	std::memcpy(data.data(), cores_usage_data.data(), cores_usage_length);
	offset += cores_usage_length;

	std::memcpy(data.data() + offset, &cpu_usage, cpu_usage_length);
    offset += cpu_usage_length;

	std::memcpy(data.data() + offset, &ram_usage, ram_usage_length);
    offset += ram_usage_length;

	std::memcpy(data.data() + offset, &temperature, temperature_length);
    offset += temperature_length;

	std::memcpy(data.data() + offset, &heap_usage, heap_usage_length);
    offset += heap_usage_length;

	std::memcpy(data.data() + offset, &stack_usage, stack_usage_length);

	return data;
}
