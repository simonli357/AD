#include "SWLoadMsg.hpp"
#include "ros/serialization.h"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <cstdio>
#include <dirent.h>
#include <filesystem>
#include <fstream>
#include <hwloc.h>
#include <malloc.h>
#include <regex>
#include <sstream>
#include <sys/resource.h>
#include <unistd.h>
#include <utility>
#include <vector>

void SWLoadMsg::refresh() {
	get_cores_usage();
	get_ram_usage();
	get_temperature();
	get_heap_usage();
	get_stack_usage();
	encode();
}

uint32_t SWLoadMsg::compute_lengths_length() { return lengths_length; }

uint32_t SWLoadMsg::compute_data_length() { return data_length; }

std::vector<uint8_t> SWLoadMsg::get_lengths() {
	std::vector<uint8_t> lengths(lengths_length);
	std::memcpy(lengths.data(), &lengths_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length, &cores_usage_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 2, &ram_usage_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 3, &temperature_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 4, &heap_usage_length, bytes_length);
	std::memcpy(lengths.data() + bytes_length * 5, &stack_usage_length, bytes_length);
	return lengths;
}

std::vector<uint8_t> SWLoadMsg::get_data() {
	std::vector<uint8_t> data(data_length);

	std::vector<uint8_t> cores_usage_data = serializeFloat32MultiArray(cores_usage);

	size_t offset = 0;
	std::memcpy(data.data(), cores_usage_data.data(), cores_usage_length);
	offset += cores_usage_length;

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

void SWLoadMsg::encode() {
	cores_usage_length = ros::serialization::serializationLength(cores_usage);
	ram_usage_length = sizeof(ram_usage);
	temperature_length = sizeof(temperature);
	heap_usage_length = sizeof(heap_usage);
	stack_usage_length = sizeof(stack_usage);
	data_length = cores_usage_length + ram_usage_length + temperature_length + heap_usage_length + stack_usage_length;
}

void SWLoadMsg::get_cores_usage() {
	auto curr_stats = read_proc_stat();

	std::vector<float> utilizations;
	if (first_core_query_) {
		first_core_query_ = false;
		prev_stats_ = curr_stats;
		utilizations.resize(curr_stats.size(), 0.0);
	} else {
		for (auto &[core_id, curr] : curr_stats) {
			auto it = prev_stats_.find(core_id);
			if (it == prev_stats_.end()) {
				utilizations.push_back(0.0);
				continue;
			}
			auto &prev = it->second;
			uint64_t total_diff = curr.total - prev.total;
			uint64_t idle_diff = curr.idle - prev.idle;

			double usage = 0.0;
			if (total_diff > 0) {
				usage = double(total_diff - idle_diff) / double(total_diff);
			}
			utilizations.push_back(usage);
		}
		prev_stats_ = std::move(curr_stats);
	}

	std_msgs::Float32MultiArray result;
	result.data = std::move(utilizations);
	cores_usage = std::move(result);
}

void SWLoadMsg::get_ram_usage() {
	std::ifstream meminfo("/proc/meminfo");
	std::string line;
	long mem_total = 0;
	long mem_available = 0;

	while (std::getline(meminfo, line)) {
		if (line.find("MemTotal:") == 0) {
			std::istringstream iss(line.substr(9));
			iss >> mem_total;
		} else if (line.find("MemAvailable:") == 0) {
			std::istringstream iss(line.substr(13));
			iss >> mem_available;
		}
	}

	if (mem_total <= 0 || mem_available < 0) {
		return;
	}

	ram_usage = static_cast<float>(mem_total - mem_available) / mem_total;
}

void SWLoadMsg::get_temperature() {
	// Check all hwmon devices
	const std::string hwmon_dir = "/sys/class/hwmon";
	for (const auto &entry : std::filesystem::directory_iterator(hwmon_dir)) {
		std::string hwmon_path = entry.path().string();

		// Read the sensor name
		std::ifstream name_file(hwmon_path + "/name");
		std::string sensor_name;
		std::getline(name_file, sensor_name);

		// Skip non-CPU sensors
		if (sensor_name != "coretemp" && sensor_name != "k10temp" && sensor_name != "cpu_thermal" && sensor_name != "nvme") {
			continue;
		}

		// Check all temperature inputs for this sensor
		for (int i = 1;; i++) {
			std::string temp_path = hwmon_path + "/temp" + std::to_string(i) + "_input";
			std::ifstream temp_file(temp_path);

			if (!temp_file.good())
				break;

			try {
				long temp_millic;
				temp_file >> temp_millic;
				temperature = temp_millic / 1000.0f;
				return;
			} catch (...) {
				continue;
			}
		}
	}

	// Fallback to thermal zones (common on ARM/RPi)
	const std::regex thermal_zone_re("thermal_zone\\d+");
	for (const auto &entry : std::filesystem::directory_iterator("/sys/class/thermal")) {
		if (std::regex_match(entry.path().filename().string(), thermal_zone_re)) {
			std::ifstream type_file(entry.path().string() + "/type");
			std::string zone_type;
			std::getline(type_file, zone_type);

			if (zone_type == "x86_pkg_temp" || zone_type == "cpu-thermal" || zone_type.find("cpu") != std::string::npos) {
				std::ifstream temp_file(entry.path().string() + "/temp");
				long temp_millic;
				temp_file >> temp_millic;
				temperature = temp_millic / 1000.0f;
				return;
			}
		}
	}

	temperature = -1.0f; // No valid sensor found
}

void SWLoadMsg::get_heap_usage() {
	// Get process memory stats from /proc/self/status
	std::ifstream status_file("/proc/self/status");
	std::string line;
	unsigned long vmhwm = 0;  // Peak resident set size ("high water mark")
	unsigned long vmsize = 0; // Virtual memory size

	while (std::getline(status_file, line)) {
		if (line.substr(0, 6) == "VmHWM:") {
			std::istringstream iss(line.substr(6));
			iss >> vmhwm; // kB
		} else if (line.substr(0, 7) == "VmSize:") {
			std::istringstream iss(line.substr(7));
			iss >> vmsize; // kB
		}
	}

	// Convert to bytes
	vmhwm *= 1024;
	vmsize *= 1024;

	if (vmsize == 0) {
		heap_usage = 0.0f;
		return;
	}
	heap_usage = static_cast<float>(vmhwm) / vmsize; // Physical usage vs. virtual allocation
}

void SWLoadMsg::get_stack_usage() {
	// Get stack size limit
	struct rlimit stack_limits;
	getrlimit(RLIMIT_STACK, &stack_limits);
	size_t stack_size = stack_limits.rlim_cur;

	// Get current stack pointer (RSP for x86_64)
	void *stack_ptr;

#if defined(__x86_64__) || defined(__i386__)
	asm volatile("mov %%rsp, %0" : "=r"(stack_ptr));
#elif defined(__aarch64__)
	asm volatile("mov %0, sp" : "=r"(stack_ptr));
#else
#error "Unsupported architecture"
#endif

	// Get thread's stack base and size (corrected for downward-growing stacks)
	pthread_attr_t attr;
	void *stack_base_low; // Lowest address of the stack
	size_t actual_stack_size;
	pthread_getattr_np(pthread_self(), &attr);
	pthread_attr_getstack(&attr, &stack_base_low, &actual_stack_size);
	pthread_attr_destroy(&attr);

	// Calculate stack high address (initial position)
	void *stack_high = (char *)stack_base_low + actual_stack_size;

	// Used bytes = distance from current SP to stack high address
	uintptr_t used_bytes = (uintptr_t)stack_high - (uintptr_t)stack_ptr;

	stack_usage = static_cast<float>(used_bytes) / actual_stack_size;
}
