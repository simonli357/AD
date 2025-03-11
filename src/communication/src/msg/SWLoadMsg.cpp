#include "SWLoadMsg.hpp"
#include "ros/serialization.h"
#include "std_msgs/Float64MultiArray.h"
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
#include <thread>
#include <unistd.h>
#include <vector>

SWLoadMsg::SWLoadMsg() {}

SWLoadMsg::SWLoadMsg(std_msgs::Float64MultiArray &cores_usage, float ram_usage, float temperature, float heap_usage, float stack_usage)
	: cores_usage(cores_usage), ram_usage(ram_usage), temperature(temperature), heap_usage(heap_usage), stack_usage(stack_usage) {
	cores_usage_length = ros::serialization::serializationLength(cores_usage);
	ram_usage_length = sizeof(ram_usage);
	temperature_length = sizeof(temperature);
	heap_usage_length = sizeof(heap_usage);
	stack_usage_length = sizeof(stack_usage);
	data_length = cores_usage_length + ram_usage_length + temperature_length + heap_usage_length + stack_usage_length;
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

	std::vector<uint8_t> cores_usage_data = serializeFloat64MultiArray(cores_usage.value());

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

std_msgs::Float64MultiArray SWLoadMsg::get_cores_usage() {
	hwloc_topology_t topology;
	hwloc_topology_init(&topology);
	hwloc_topology_load(topology);

	// Get core count and OS indices
	int depth = hwloc_get_type_depth(topology, HWLOC_OBJ_PU);
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
				double utilization = 1.0 * (total_diff - idle_diff) / total_diff;
				std::cout << utilization << std::endl;
				utilizations.push_back(utilization);
			}
		}
	}

	hwloc_topology_destroy(topology);
	std_msgs::Float64MultiArray results;
	results.data = utilizations;
	return results;
}

float SWLoadMsg::get_ram_usage() {
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
		return -1.0f; // Error value
	}

	return static_cast<float>(mem_total - mem_available) / mem_total;
}

float SWLoadMsg::get_temperature() {
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
				return temp_millic / 1000.0f;
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
				return temp_millic / 1000.0f;
			}
		}
	}

	return -1.0f; // No valid sensor found
}

float SWLoadMsg::get_heap_usage() {
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

	if (vmsize == 0)
		return 0.0f;
	return static_cast<float>(vmhwm) / vmsize; // Physical usage vs. virtual allocation
}

float SWLoadMsg::get_stack_usage() {
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

	return static_cast<float>(used_bytes) / actual_stack_size;
}
