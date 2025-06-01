#include <mbed.h>
#include <periodics/blinker.hpp>
#include <periodics/imu.hpp>
// #include <drivers/bno055_c.hpp>
#include <periodics/instantconsumption.hpp>
#include <periodics/totalvoltage.hpp>
#include <drivers/velocitycontrolduration.hpp>
#include <drivers/serialmonitor.hpp>
#include <brain/robotstatemachine.hpp>
#include <utils/taskmanager.hpp>
#include <utils/task.hpp>
#include <drivers/steeringmotor.hpp>
#include <periodics/encoder.hpp>
#include "rtos.h"
#include <drivers/dcmotor.hpp>
#include "mbed_power_mgmt.h"

BufferedSerial g_rpi(USBTX, USBRX, 115200);

#define PIN_INA  D7
#define PIN_INB  D5
#define PIN_PWM  D10
#define SPEEDING_DRIVER_PWM_PIN D3
#define STEERING_DRIVER_PWM_PIN D4

drivers::CMotorDriverVnh g_motorVnhDriver(PIN_PWM, PIN_INA, PIN_INB,
                                                    -0.30f, 0.30f);
periodics::CBlinker g_blinker(1, LED1);
periodics::CTotalVoltage g_totalvoltage(1, A1, g_rpi);
periodics::CImu g_imu(1, g_rpi, I2C_SDA, I2C_SCL);
periodics::CEncoder g_encoder(1, 1, g_rpi, D2);
drivers::CSpeedingMotor g_speedingDriver(1,g_rpi,SPEEDING_DRIVER_PWM_PIN, g_encoder); //speed in cm/s
drivers::CSteeringMotor g_steeringDriver(1, g_rpi, STEERING_DRIVER_PWM_PIN, g_imu, g_speedingDriver);
brain::CRobotStateMachine g_robotstatemachine(1, g_rpi, g_steeringDriver, g_speedingDriver);

drivers::SerialSubscriberMap g_serialMonitorSubscribers = {
    {"10",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackPWMcommand)},
    {"11",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackComputecommand)},
    {"12",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackPIDcommand)},
    {"13",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSetcommand)},
    {"15",mbed::callback(&g_motorVnhDriver,&drivers::CMotorDriverVnh::serialCallbackMOTOR)},
    {"3",mbed::callback(&g_imu,&periodics::CImu::recalibrateYaw)},
};
drivers::SerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

static Thread blinkerThread(osPriorityLow,    1024, nullptr, "blinker");
static Thread imuThread    (osPriorityHigh, 2048, nullptr, "imu");
static Thread encoderThread(osPriorityHigh,   2048, nullptr, "encoder");
static Thread totalVoltageThread(osPriorityNormal, 2048, nullptr, "totalVoltage");
void blinkerTask() {
    while (true) {
        g_blinker.run();
        ThisThread::sleep_for(500ms);
    }
}
void imuTask() {
    while (true) {
        g_imu.run();
        g_steeringDriver.run();
        ThisThread::sleep_for(50ms);
    }
}
void encoderTask() {
    while (true) {
        g_encoder.run();
        ThisThread::sleep_for(2ms);
    }
}
void totalVoltageTask() {
    while (true) {
        g_totalvoltage.run();
        ThisThread::sleep_for(100ms);
    }
}

void startupMessage() {
    g_rpi.write("\r\n\r\n", 4);
    g_rpi.write("#################\r\n", 19);
    g_rpi.write("#   I'm alive   #\r\n", 19);
    g_rpi.write("#################\r\n", 19);
    g_rpi.write("\r\n", 2);
}

int main() 
{
    startupMessage();
    sleep_manager_lock_deep_sleep();
    blinkerThread.start(blinkerTask);
    imuThread.start(imuTask);
    encoderThread.start(encoderTask);
    g_serialMonitor.start(5ms);
    ThisThread::sleep_for(osWaitForever);
}
