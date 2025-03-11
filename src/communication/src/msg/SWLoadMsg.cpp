#include "SWLoadMsg.hpp"
#include "ros/serialization.h"
#include <fstream>
#include <hwloc.h>
#include <sstream>
#include <thread>
#include <vector>

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

std::unordered_map<int, SWLoadMsg::CoreUsage> SWLoadMsg::read_proc_stat() {
	std::ifstream stat_file("/proc/stat");
	std::string line;
	std::unordered_map<int, CoreUsage> core_usages;

	while (std::getline(stat_file, line)) {
		if (line.substr(0, 3) == "cpu") {
			std::istringstream iss(line);
			std::string cpu_label;
			iss >> cpu_label;

			if (cpu_label == "cpu")
				continue; // Skip aggregate

			int core_id = std::stoi(cpu_label.substr(3));
			unsigned long long user, nice, system, idle, iowait;
			iss >> user >> nice >> system >> idle >> iowait;

			core_usages[core_id] = {user + nice + system + idle + iowait, idle + iowait};
		}
	}
	return core_usages;
}

std_msgs::Float64MultiArray SWLoadMsg::get_cores_usage() {
	hwloc_topology_t topology;
	hwloc_topology_init(&topology);
	hwloc_topology_load(topology);

	// Get core count and OS indices
	int depth = hwloc_get_type_depth(topology, HWLOC_OBJ_CORE);
	int cores = hwloc_get_nbobjs_by_depth(topology, depth);

	// First measurement
	auto prev_stats = read_proc_stat();
	std::this_thread::sleep_for(std::chrono::milliseconds(1000));
	auto curr_stats = read_proc_stat();

	std::vector<double> utilizations;
	for (int i = 0; i < cores; i++) {
		hwloc_obj_t core = hwloc_get_obj_by_depth(topology, depth, i);
		int os_index = core->os_index;

		if (prev_stats.count(os_index) && curr_stats.count(os_index)) {
			auto &prev = prev_stats[os_index];
			auto &curr = curr_stats[os_index];

			unsigned long long total_diff = curr.total - prev.total;
			unsigned long long idle_diff = curr.idle - prev.idle;

			if (total_diff > 0) {
				double utilization = 100.0 * (total_diff - idle_diff) / total_diff;
				utilizations.push_back(utilization);
			}
		}
	}

	hwloc_topology_destroy(topology);
    std_msgs::Float64MultiArray results;
    results.data = utilizations;
    return results;
}
