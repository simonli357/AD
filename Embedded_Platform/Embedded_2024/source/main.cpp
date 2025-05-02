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
#include <periodics/serialPrinter.hpp>
#include "rtos.h"

const float g_baseTick = 0.0001f; // seconds

// Serial interface with the another device(like single board computer). It's an built-in class of mbed based on the UART communication, the inputs have to be transmitter and receiver pins. 
UnbufferedSerial g_rpi(USBTX, USBRX, 460800); // baud rate 460800

periodics::CBlinker g_blinker(0.5 / g_baseTick, LED1);
// periodics::CInstantConsumption g_instantconsumption(0.2 / g_baseTick, A2, g_rpi);
periodics::CTotalVoltage g_totalvoltage(3.0 / g_baseTick, A1, g_rpi);
periodics::CImu g_imu(0.1/ g_baseTick, g_rpi, I2C_SDA, I2C_SCL);
periodics::CEncoder g_encoder(0.01/g_baseTick, g_baseTick, g_rpi, D2);
drivers::CSpeedingMotor g_speedingDriver(0.1/g_baseTick,g_rpi,D3, g_encoder); //speed in cm/s
drivers::CSteeringMotor g_steeringDriver(0.1 / g_baseTick, g_rpi, D4, g_imu, g_speedingDriver);
// drivers::CVelocityControlDuration g_velocityControlDuration(0.1/g_baseTick, g_steeringDriver, g_speedingDriver);
brain::CRobotStateMachine g_robotstatemachine(0.1/g_baseTick, g_rpi, g_steeringDriver, g_speedingDriver);
// periodics::CSerialPrinter g_serialPrinter(0.1/g_baseTick, g_rpi);

drivers::CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    // {"1",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSPEEDcommand)},
    // {"2",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSTEERcommand)},
    // {"3",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackBRAKEcommand)},
    // {"4",mbed::callback(&g_motorCalibration,&periodics::CTotalVoltage::SpeedMotorCalibration)},
    // {"5",mbed::callback(&g_totalvoltage,&periodics::CTotalVoltage::TotalPublisherCommand)},
    // {"6",mbed::callback(&g_instantconsumption,&periodics::CInstantConsumption ::InstantPublisherCommand)},
    {"7",mbed::callback(&g_imu,&periodics::CImu::ImuPublisherCommand)},
    // {"8",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackBOTHcommand)},
    // {"8",mbed::callback(&g_complexMoves, &drivers::CComplexMoves::serialCallbackComplexMovesCommand)},
    // {"9",mbed::callback(&g_velocityControlDuration, &drivers::CVelocityControlDuration::serialCallbackVCDCommand)},
    // Callback for the PWM input command
    {"10",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackPWMcommand)},
    {"11",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackComputecommand)},
    {"12",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackPIDcommand)},
    {"13",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSetcommand)}
};

// Create the serial monitor object, which decodes, redirects the messages and transmits the responses.
drivers::CSerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

static Thread blinkerThread(osPriorityLow,    1024, nullptr, "blinker");
static Thread imuThread    (osPriorityNormal, 2048, nullptr, "imu");
static Thread encoderThread(osPriorityHigh,   2048, nullptr, "encoder");
static Thread serialMonThread(osPriorityNormal,2048,nullptr,"serialMon");
static Thread stateMachineThread(osPriorityAboveNormal,4096,nullptr,"stateMachine");
void blinkerTask() {
    while (true) {
        g_blinker.run();
        ThisThread::sleep_for(500ms);
    }
}
void imuTask() {
    while (true) {
        g_imu.run();
        ThisThread::sleep_for(100ms);
    }
}
void encoderTask() {
    while (true) {
        g_encoder.run();
        ThisThread::sleep_for(10ms);
    }
}
void serialMonitorTask() {
    while (true) {
        g_serialMonitor.run();
        ThisThread::sleep_for(10ms);
    }
}
void stateMachineTask() {
    while (true) {
        g_robotstatemachine.run();
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

utils::CTask* g_taskList[] = {
    &g_blinker,
    &g_imu,
    &g_robotstatemachine,
    &g_serialMonitor,
    &g_encoder,
}; 

utils::CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(utils::CTask*), g_baseTick);

/**
 * @brief Setup function for initializing some objects and transmitting a startup message through the serial. 
 * 
 * @return uint32_t Error level codes error's type.
 */
uint32_t setup()
{
    g_rpi.write("\r\n\r\n", 4);
    g_rpi.write("#################\r\n", 19);
    g_rpi.write("#               #\r\n", 19);
    g_rpi.write("#   I'm alive   #\r\n", 19);
    g_rpi.write("#               #\r\n", 19);
    g_rpi.write("#################\r\n", 19);
    g_rpi.write("\r\n", 2);
    return 0;    
}

/**
 * @brief Loop function has aim to apply repeatedly task
 * 
 * @return uint32_t Error level codes error's type.
 */
uint32_t loop()
{
    g_taskManager.mainCallback();
    return 0;
}

/**
 * @brief Main function applies the setup function and the loop function periodically. It runs automatically after the board started.
 * 
 * @return int Error level codes error's type.  
 */
int main() 
{
    startupMessage();
    blinkerThread.start(blinkerTask);
    imuThread.start(imuTask);
    encoderThread.start(encoderTask);
    serialMonThread.start(serialMonitorTask);
    stateMachineThread.start(stateMachineTask);
    ThisThread::sleep_for(osWaitForever);
}