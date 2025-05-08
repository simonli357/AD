#pragma once

#include "Encoder.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <hwloc.h>
#include <unordered_map>
#include <vector>

class SWLoadMsg : public Encoder {
  public:
	SWLoadMsg() = default;
	SWLoadMsg(SWLoadMsg &&) = default;
	SWLoadMsg(const SWLoadMsg &) = default;
	SWLoadMsg &operator=(SWLoadMsg &&) = delete;
	SWLoadMsg &operator=(const SWLoadMsg &) = delete;
	~SWLoadMsg() = default;

	std_msgs::Float32MultiArray cores_usage;
	float ram_usage;
	float temperature;
	float heap_usage;
	float stack_usage;

	struct CoreUsage {
		uint64_t total;
		uint64_t idle;
	};
	std::unordered_map<int, CoreUsage> prev_stats_;
	bool first_core_query_ = true;

	void refresh();

  private:
	void encode();
	void get_cores_usage();
	void get_ram_usage();
	void get_temperature();
	void get_heap_usage();
	void get_stack_usage();

	const size_t bytes_length = 4;
	const size_t num_elements = 5;
	uint32_t lengths_length = (num_elements + 1) * bytes_length;
	uint32_t data_length;
	uint32_t cores_usage_length;
	uint32_t ram_usage_length;
	uint32_t temperature_length;
	uint32_t heap_usage_length;
	uint32_t stack_usage_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
	std::unordered_map<int, CoreUsage> read_proc_stat();
};
