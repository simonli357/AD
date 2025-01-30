#pragma once

#include <cstdint>
#include <cstring>
#include <memory>
#include <sstream>
#include <string>
#include <vector>

template <typename T> class Decoder {
  public:
	Decoder() = default;
	Decoder(Decoder &&) = default;
	Decoder(const Decoder &) = default;
	Decoder &operator=(Decoder &&) = delete;
	Decoder &operator=(const Decoder &) = delete;
	~Decoder() = default;

	const size_t bytes_length = 4;
	std::vector<std::vector<uint8_t>> split(std::vector<uint8_t> &bytes);
	virtual std::unique_ptr<T> deserialize(std::vector<uint8_t> &bytes);

	int32_t int32_t_from_bytes(std::vector<uint8_t> &bytes);
	double double_from_bytes(std::vector<uint8_t> &bytes);
	float float_from_bytes(std::vector<uint8_t> &bytes);
	bool bool_from_bytes(std::vector<uint8_t> &bytes);
};

template <typename T>
std::vector<std::vector<uint8_t>> Decoder<T>::split(std::vector<uint8_t> &bytes) {
	uint32_t lengths_length = 0;
	std::memcpy(&lengths_length, &bytes[0], bytes_length);

	size_t num_elements = (lengths_length - bytes_length) / bytes_length;
	std::vector<std::vector<uint8_t>> splits(num_elements);
	size_t num_bytes = bytes.size();

	size_t size_offset = bytes_length;
	size_t data_offset = lengths_length;
	for (size_t i = 0; i < num_elements; i++) {
		// Get the size of the element
		uint32_t size = 0;
		std::memcpy(&size, &bytes[size_offset], bytes_length);
		size_offset += bytes_length;
		// Read the data and append
		std::vector<uint8_t> split(size);
		std::memcpy(split.data(), &bytes[data_offset], size);
		splits[i] = std::move(split);
		data_offset += size;
	}

	return splits;
}

template <typename T>
int32_t Decoder<T>::int32_t_from_bytes(std::vector<uint8_t> &bytes) {
    uint32_t i;
    std::memcpy(&i, bytes.data(), bytes.size());
    return i;
}

template <typename T>
double Decoder<T>::double_from_bytes(std::vector<uint8_t> &bytes) {
	std::string double_str(bytes.begin(), bytes.end());
    double d;
    std::istringstream(double_str) >> d;
    return d;
}

template <typename T>
float Decoder<T>::float_from_bytes(std::vector<uint8_t> &bytes) {
	std::string float_str(bytes.begin(), bytes.end());
    float d;
    std::istringstream(float_str) >> d;
    return d;
}

template <typename T>
bool Decoder<T>::bool_from_bytes(std::vector<uint8_t> &bytes) {
	std::string bool_str(bytes.begin(), bytes.end());
    return bool_str == "true";
}

template <typename T>
std::unique_ptr<T> Decoder<T>::deserialize(std::vector<uint8_t> &bytes) { return nullptr; }
