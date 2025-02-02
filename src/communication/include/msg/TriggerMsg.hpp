#pragma once

#include "Decoder.hpp"
#include "Encoder.hpp"
#include "std_srvs/Trigger.h"
#include "std_srvs/TriggerRequest.h"
#include "std_srvs/TriggerResponse.h"
#include <cstdint>
#include <optional>
#include <vector>

class TriggerMsg : public Decoder<TriggerMsg>, public Encoder {
  public:
	TriggerMsg();
	TriggerMsg(std_srvs::Trigger &trigger);
	TriggerMsg(std_srvs::TriggerResponse &response);
	TriggerMsg(std_srvs::TriggerRequest &request, std_srvs::TriggerResponse &response);
	TriggerMsg(TriggerMsg &&) = default;
	TriggerMsg(const TriggerMsg &) = default;
	TriggerMsg &operator=(TriggerMsg &&) = delete;
	TriggerMsg &operator=(const TriggerMsg &) = delete;
	~TriggerMsg() = default;

	std::optional<std_srvs::Trigger> trigger;
	std::optional<std_srvs::TriggerRequest> request;
	std::optional<std_srvs::TriggerResponse> response;

	std::unique_ptr<TriggerMsg> deserialize(std::vector<uint8_t> &bytes) override;

  private:
	const size_t num_elements = 2;
	uint32_t lengths_length = num_elements * (bytes_length + 1);
	uint32_t data_length;
	uint32_t request_length;
	uint32_t response_length;
	uint32_t compute_lengths_length() override;
	uint32_t compute_data_length() override;
	std::vector<uint8_t> get_lengths() override;
	std::vector<uint8_t> get_data() override;
};
