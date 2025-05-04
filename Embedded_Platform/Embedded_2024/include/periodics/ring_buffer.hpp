// ring_buffer.hpp
#pragma once

#include <cstdint>
#include <cstddef>

/**
 * @brief Packet types for telemetry messages
 */
enum class PacketType : uint8_t {
    Encoder = 0x01,
    Imu     = 0x02,
    Combined = 0x03,   // <— new
};

/**
 * @brief A unified telemetry message for encoder or IMU data.
 *        Packed tightly for binary transport.
 */
#pragma pack(push,1)
struct TelemetryMsg {
    PacketType type;   ///< 1 byte: message kind
    uint32_t   ts_us;  ///< 4 bytes: timestamp in microseconds
    union {
        struct {
            int32_t angle_hundredths;
            int32_t speed_hundredths;  ///< deg/s × 100
        } encoder;
        struct {
            int32_t yaw_h;    ///< deg × 100
            int32_t pitch_h;  ///< deg × 100
            int32_t roll_h;   ///< deg × 100
            int32_t ax_mg;    ///< mg × 1000
            int32_t ay_mg;    ///< mg × 1000
            int32_t az_mg;    ///< mg × 1000
            int32_t gx_mrs;   ///< mrs × 1000
            int32_t gy_mrs;   ///< mrs × 1000
            int32_t gz_mrs;   ///< mrs × 1000
        } imu;
    } data;
};
#pragma pack(pop)

// Buffer size (must be a power of two for mask wrapping)
static constexpr std::size_t RB_SIZE = 2048;

/**
 * @brief Push a telemetry message into the ring buffer.
 * @param msg  The message to enqueue.
 * @return     true if enqueued; false if the buffer was full.
 */
bool rb_push(const TelemetryMsg &msg);

/**
 * @brief Pop the oldest telemetry message from the ring buffer.
 * @param msg  Reference to receive the message.
 * @return     true if a message was dequeued; false if the buffer was empty.
 */
bool rb_pop(TelemetryMsg &msg);

