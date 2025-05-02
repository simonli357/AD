// serial_printer_task.cpp
#include "serialPrinter.hpp"
#include <cstring>

static constexpr uint8_t SOF = 0xAA;

namespace periodics {

    #pragma pack(push,1)

    struct CombinedMsg {
        int16_t angle_hundredths;       // last encoder angle
        int32_t avg_speed_hundredths;   // average of all speeds
        // then the IMU block exactly as in TelemetryMsg:
        int16_t yaw_h, pitch_h, roll_h;
        int16_t ax_mg, ay_mg, az_mg;
        int16_t gx_mrs, gy_mrs, gz_mrs;
    };

    #pragma pack(pop)
    static_assert(sizeof(CombinedMsg) == 24, "CombinedMsg must be 24 bytes");

    void CSerialPrinter::_run() {
        TelemetryMsg msg;
    
        // accumulators
        int32_t sumSpeed = 0;
        uint32_t countSpeed = 0;
        int16_t lastAngle = 0;
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
        cmb.avg_speed_hundredths = countSpeed ? int64_t(sumSpeed / countSpeed) : 0;
        // copy the IMU fields
        cmb.yaw_h   = lastImu.yaw_h;
        cmb.pitch_h = lastImu.pitch_h;
        cmb.roll_h  = lastImu.roll_h;
        cmb.ax_mg   = lastImu.ax_mg;
        cmb.ay_mg   = lastImu.ay_mg;
        cmb.az_mg   = lastImu.az_mg;
        cmb.gx_mrs  = lastImu.gx_mrs;
        cmb.gy_mrs  = lastImu.gy_mrs;
        cmb.gz_mrs  = lastImu.gz_mrs;
    
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
        printf("[DBG] raw_h = %d  (bytes = 0x%04X)\n",
            raw_h,
            static_cast<uint32_t>(raw_h) & 0xFFFF);
        printf("speed/PL = %d (bytes = 0x%04X)\n",
                cmb.avg_speed_hundredths,
                static_cast<uint32_t>(cmb.avg_speed_hundredths) & 0xFFFF);
        printf("angle/PL = %d (bytes = 0x%04X)\n",
                cmb.angle_hundredths,
                static_cast<uint32_t>(cmb.angle_hundredths) & 0xFFFF);

        printf("[PRT] Combined payload length = %u bytes\n", sizeof(cmb));
        m_serial.write(reinterpret_cast<const char*>(header), 3);
        m_serial.write(reinterpret_cast<const char*>(&cmb), sizeof(cmb));
        m_serial.write(reinterpret_cast<const char*>(crcBytes), 2);
    }
    

} // namespace periodics
