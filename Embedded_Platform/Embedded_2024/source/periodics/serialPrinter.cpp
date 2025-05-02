// serial_printer_task.cpp
#include "serialPrinter.hpp"
#include <cstring>

static constexpr uint8_t SOF = 0xAA;

namespace periodics {

    #pragma pack(push,1)

    struct CombinedMsg {
        int32_t angle_hundredths;       // last encoder angle
        int32_t avg_speed_hundredths;   // average of all speeds
        // then the IMU block exactly as in TelemetryMsg:
        int32_t yaw_h, pitch_h;
    };

    #pragma pack(pop)
    static_assert(sizeof(CombinedMsg) == 16, "CombinedMsg must be 16 bytes");

    void CSerialPrinter::_run() {
        TelemetryMsg msg;

        // ————— One-time Timer setup
        static Timer execTimer;
        static bool timerStarted = false;
        if (!timerStarted) {
            execTimer.start();
            timerStarted = true;
        }

        // ————— 1) Mark start of this execution
        uint32_t start_us = execTimer.read_us();

        // accumulators
        int32_t sumSpeed = 0;
        int32_t countSpeed = 0;
        int32_t lastAngle = 0;
        bool gotImu = false;
        decltype(msg.data.imu) lastImu = {};
    
        // 1) drain everything, but don’t send yet
        while (rb_pop(msg)) {
            if (msg.type == PacketType::Encoder) {
                lastAngle = msg.data.encoder.angle_hundredths;
                sumSpeed += msg.data.encoder.speed_hundredths;
                ++countSpeed;
            }
            else if (msg.type == PacketType::Imu) {
                lastImu = msg.data.imu;
                gotImu = true;
            }
        }

        // nothing new? bail out
        if (countSpeed == 0 && !gotImu) return;
    
        // 2) build the combined payload
        CombinedMsg cmb;
        cmb.angle_hundredths     = lastAngle;
        cmb.avg_speed_hundredths = countSpeed ? int32_t(sumSpeed / countSpeed) : 0;
        // copy the IMU fields
        cmb.yaw_h   = lastImu.yaw_h;
        cmb.pitch_h = lastImu.pitch_h;
    
        // 3) frame & send one Combined packet
        uint8_t header[3] = {
          0xAA,
          static_cast<uint8_t>(PacketType::Combined),
          uint8_t(sizeof(CombinedMsg))
        };
    
        // CRC over [Type|Len|Payload]
        uint8_t crcBuf[2 + sizeof(CombinedMsg)];
        crcBuf[0] = header[1];
        crcBuf[1] = header[2];
        memcpy(crcBuf+2, &cmb, sizeof(cmb));
        uint16_t crc = computeCRC16(crcBuf, sizeof(crcBuf));
        uint8_t crcBytes[2] = { uint8_t(crc & 0xFF), uint8_t(crc >> 8) };
        int32_t raw_h = cmb.avg_speed_hundredths;
        
        // Debug print
        // printf("[DBG] raw_h = %d  (bytes = 0x%04X)\n",
        //     raw_h,
        //     static_cast<uint32_t>(raw_h) & 0xFFFF);
        // printf("speed/PL = %d (bytes = 0x%04X)\n",
        //         cmb.avg_speed_hundredths,
        //         static_cast<uint32_t>(cmb.avg_speed_hundredths) & 0xFFFF);
        // printf("angle/PL = %d (bytes = 0x%04X)\n",
        //         cmb.angle_hundredths,
        //         static_cast<uint32_t>(cmb.angle_hundredths) & 0xFFFF);

        // printf("[PRT] Combined payload length = %u bytes\n", sizeof(cmb));
        // m_serial.write(reinterpret_cast<const char*>(header), 3);
        // m_serial.write(reinterpret_cast<const char*>(&cmb), sizeof(cmb));
        // m_serial.write(reinterpret_cast<const char*>(crcBytes), 2);

        // ————— 5) Mark end of execution and accumulate for average print
        uint32_t end_us     = execTimer.read_us();
        uint32_t elapsed_us = end_us - start_us;

        static uint64_t sum_exec   = 0;
        static uint32_t count_exec = 0;
        static uint64_t sum_interval_us = 0;
        static uint32_t count_interval  = 0;
        static uint32_t prevStart_us    = 0;

        // Accumulate period statistics (unchanged)
        if (prevStart_us != 0) {
            uint32_t delta_start = start_us - prevStart_us;
            sum_interval_us += delta_start;
            count_interval++;
        }
        prevStart_us = start_us;

        sum_exec   += elapsed_us;
        count_exec += 1;

        constexpr uint32_t AVG_N = 200;
        if (count_exec >= AVG_N) {
            uint32_t avg_exec     = sum_exec / AVG_N;
            uint32_t avg_interval = (count_interval > 0)
                                    ? static_cast<uint32_t>(sum_interval_us / count_interval)
                                    : 0;

            printf("\n[Telemetry] avg exec = %u µs, avg period = %u µs over %u runs\n",
                avg_exec, avg_interval, AVG_N);

            sum_exec         = 0;
            count_exec       = 0;
            sum_interval_us  = 0;
            count_interval   = 0;
        }
    }
    

} // namespace periodics
