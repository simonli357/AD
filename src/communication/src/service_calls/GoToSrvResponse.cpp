#include "service_calls/GoToSrvResponse.hpp"
#include "std_msgs/Float32MultiArray.h"
#include <cstdint>
#include <cstring>
#include <netinet/in.h>
#include <vector>

using std_msgs::Float32MultiArray;

GoToSrvResponse::GoToSrvResponse(Float32MultiArray &state_refs, Float32MultiArray &input_refs, Float32MultiArray &wp_attributes, Float32MultiArray &wp_normals)
	: state_refs(state_refs), input_refs(input_refs), wp_attributes(wp_attributes), wp_normals(wp_normals) {}

uint32_t GoToSrvResponse::compute_lengths_length() {
    return lengths_length;
}

uint32_t GoToSrvResponse::compute_data_length() {
    return data_length;
}

std::vector<uint8_t> GoToSrvResponse::get_lengths() {
    std::vector<uint8_t> lengths(lengths_length);
    
    uint32_t big_endian_state_refs_length = htonl(state_refs_length);
    uint32_t big_endian_input_refs_length = htonl(input_refs_length);
    uint32_t big_endian_wp_attributes_length = htonl(wp_attributes_length);
    uint32_t big_endian_wp_normals_length = htonl(wp_normals_length);

    std::memcpy(lengths.data(), &big_endian_state_refs_length, length_bytes);
    std::memcpy(lengths.data() + length_bytes, &big_endian_input_refs_length, length_bytes);
    std::memcpy(lengths.data() + length_bytes * 2, &big_endian_wp_attributes_length, length_bytes);
    std::memcpy(lengths.data() + length_bytes * 3, &big_endian_wp_normals_length, length_bytes);

    return lengths;
}

std::vector<uint8_t> GoToSrvResponse::get_data() {
    std::vector<uint8_t> data(data_length);

    std::vector<uint8_t> state_refs_data = serializeFloat32MultiArray(state_refs_length, state_refs);
    std::vector<uint8_t> input_refs_data = serializeFloat32MultiArray(input_refs_length, input_refs);
    std::vector<uint8_t> wp_attributes_data = serializeFloat32MultiArray(wp_attributes_length, wp_attributes);
    std::vector<uint8_t> wp_normals_data = serializeFloat32MultiArray(wp_normals_length, wp_normals);
    
    size_t offset = 0;
    std::memcpy(data.data(), state_refs_data.data(), state_refs_length);
    offset += state_refs_length;

    std::memcpy(data.data() + offset, input_refs_data.data(), input_refs_length);
    offset += input_refs_length;

    std::memcpy(data.data() + offset, wp_attributes_data.data(), wp_attributes_length);
    offset += wp_attributes_length;

    std::memcpy(data.data() + offset, wp_normals_data.data(), wp_normals_length);

    return data;
}
