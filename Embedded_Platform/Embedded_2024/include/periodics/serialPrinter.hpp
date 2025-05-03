// serial_printer_task.hpp
#pragma once

#include "utils/task.hpp"
#include "ring_buffer.hpp"
#include "mbed.h"

namespace periodics {

/**
 * @brief A low-priority task to drain the telemetry ring buffer
 *        and send binary packets (encoder angle+speed, IMU data)
 */
class CSerialPrinter : public utils::CTask {
public:
    /**
     * @param periodTicks  Number of g_baseTick ticks between runs
     * @param serial       UART interface for output
     */
    CSerialPrinter(uint32_t periodTicks, BufferedSerial& serial)
        : utils::CTask(periodTicks), m_serial(serial) {}

    void _run() override;

private:
    BufferedSerial& m_serial;

    // Compute CRC-16-CCITT (poly 0x1021) over data
    uint16_t computeCRC16(const uint8_t* data, size_t len) {
        uint16_t crc = 0xFFFF;
        for (size_t i = 0; i < len; ++i) {
            crc ^= static_cast<uint16_t>(data[i]) << 8;
            for (int bit = 0; bit < 8; ++bit) {
                if (crc & 0x8000)
                    crc = (crc << 1) ^ 0x1021;
                else
                    crc <<= 1;
            }
        }
        return crc;
    }
};

} // namespace periodics