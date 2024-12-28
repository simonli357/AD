/**
 * Copyright (c) 2019, Bosch Engineering Center Cluj and BFMC organizers
 * All rights reserved.
 * 
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:

 * 1. Redistributions of source code must retain the above copyright notice, this
 *    list of conditions and the following disclaimer.

 * 2. Redistributions in binary form must reproduce the above copyright notice,
 *    this list of conditions and the following disclaimer in the documentation
 *    and/or other materials provided with the distribution.

 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived from
 *    this software without specific prior written permission.

 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE
*/

#ifndef IMU_H
#define IMU_H

#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX ((u8)1)
#define I2C_BUFFER_LEN 8
#define I2C0           5

/* The mbed library */
#include <mbed.h>
#include <drivers/BNO055.hpp>
/* Header file for the task manager library, which  applies periodically the fun function of it's children*/
#include <utils/task.hpp>

namespace periodics
{
   /**
    * @brief Class imu 
    * 
    * 
    */
    class CImu : public utils::CTask
    {
        public:
            /* Construnctor */
            CImu(
                uint32_t    f_period, 
                UnbufferedSerial& f_serial,
                PinName SDA,
                PinName SCL
            );
            /* Destructor */
            ~CImu();
            /* The API is used as SPI bus write */
            static s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
            /* The API is used as I2C bus read */
            static s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
            /* TThe delay routine */
            static void BNO055_delay_msek(u32 msek);
            /* Serial callback implementation */
            void ImuPublisherCommand(char const * a, char * b);
        private:
            /*I2C init routine */
            virtual void I2C_routine(void);
            /* This API is an example for reading sensor data */
            s32 bno055_data_readout_template(void);
            /* Run method */
            virtual void    _run();

            /*----------------------------------------------------------------------------*
            *  struct bno055_t parameters can be accessed by using BNO055
            *  BNO055_t having the following parameters
            *  Bus write function pointer: BNO055_WR_FUNC_PTR
            *  Bus read function pointer: BNO055_RD_FUNC_PTR
            *  Burst read function pointer: BNO055_BRD_FUNC_PTR
            *  Delay function pointer: delay_msec
            *  I2C address: dev_addr
            *  Chip id of the sensor: chip_id
            *---------------------------------------------------------------------------*/
            struct bno055_t bno055;

            /*---------------------------------------------------------------------------------------------*
            *  The static pointer variable member i2c_instance will be used inside the I2C bus APIs 
            *----------------------------------------------------------------------------------------------*/
            static I2C* i2c_instance;

            /** @brief Active flag  */
            bool            m_isActive;

            /* @brief Serial communication obj.  */
            UnbufferedSerial&      m_serial;

            float m_velocityX;
            float m_velocityY;
            float m_velocityZ;
            int m_velocityStationaryCounter;
            float init_euler_h_deg;
    }; // class CImu

}; // namespace utils

#endif // IMU_H
