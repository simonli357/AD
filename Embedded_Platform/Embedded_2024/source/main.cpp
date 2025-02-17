/* Header file for the motion controller functionality */
#include <main.hpp>

/// Base sample time for the task manager. The measurement unit of base sample time is second.
const float g_baseTick = 0.0001; // seconds

// Serial interface with the another device(like single board computer). It's an built-in class of mbed based on the UART communication, the inputs have to be transmitter and receiver pins. 
UnbufferedSerial g_rpi(USBTX, USBRX, 115200);

// It's a task for blinking periodically the built-in led on the Nucleo board, signaling the code is uploaded on the nucleo.
periodics::CBlinker g_blinker(0.5 / g_baseTick, LED1);

// // It's a task for sending periodically the instant current consumption of the battery
// periodics::CInstantConsumption g_instantconsumption(0.2 / g_baseTick, A2, g_rpi);

// // It's a task for sending periodically the battery voltage, so to notice when discharging
// periodics::CTotalVoltage g_totalvoltage(3.0 / g_baseTick, A1, g_rpi);

// It's a task for sending periodically the IMU values
periodics::CImu g_imu(0.1 / g_baseTick, g_rpi, I2C_SDA, I2C_SCL);

//PIN for a motor speed in ms, inferior and superior limit
drivers::CSpeedingMotor g_speedingDriver(D3, -50.0, 50.0); //speed in cm/s

//PIN for angle in servo degrees, inferior and superior limit
drivers::CSteeringMotor g_steeringDriver(0.05 / g_baseTick, g_rpi, D4, -100.0, 100.0, g_imu, g_speedingDriver);

// Task responsible for configuring the vehicle's speed and steering over a specified duration.
drivers::CVelocityControlDuration g_velocityControlDuration(0.1/g_baseTick, g_steeringDriver, g_speedingDriver);

// Create the motion controller, which controls the robot states and the robot moves based on the transmitted command over the serial interface. 
brain::CRobotStateMachine g_robotstatemachine(0.1/g_baseTick, g_rpi, g_steeringDriver, g_speedingDriver);

// Map for redirecting messages with the key and the callback functions. If the message key equals to one of the enumerated keys, than it will be applied the paired callback function.
drivers::CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    {"1",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSPEEDcommand)},
    {"2",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSTEERcommand)},
    {"3",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackBRAKEcommand)},
    // {"4",mbed::callback(&g_motorCalibration,&periodics::CTotalVoltage::SpeedMotorCalibration)},
    // {"5",mbed::callback(&g_totalvoltage,&periodics::CTotalVoltage::TotalPublisherCommand)},
    // {"6",mbed::callback(&g_instantconsumption,&periodics::CInstantConsumption ::InstantPublisherCommand)},
    {"7",mbed::callback(&g_imu,&periodics::CImu::ImuPublisherCommand)},
    {"8",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackBOTHcommand)},
    // {"8",mbed::callback(&g_complexMoves, &drivers::CComplexMoves::serialCallbackComplexMovesCommand)},
    {"9",mbed::callback(&g_velocityControlDuration, &drivers::CVelocityControlDuration::serialCallbackVCDCommand)},
    // Callback for the PWM input command
    {"10",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackPWMcommand)},
    {"11",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackComputecommand)},
    {"12",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackPIDcommand)},
    {"13",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSetcommand)}
};

// Create the serial monitor object, which decodes, redirects the messages and transmits the responses.
drivers::CSerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

// List of the task, each task will be applied their own periodicity, defined by the initializing the objects.
utils::CTask* g_taskList[] = {
    &g_blinker,
    // &g_instantconsumption,
    // &g_totalvoltage,
    &g_imu,
    &g_robotstatemachine,
    &g_velocityControlDuration,
    &g_serialMonitor,
    &g_steeringDriver
}; 

// Create the task manager, which applies periodically the tasks, miming a parallelism. It needs the list of task and the time base in seconds. 
utils::CTaskManager g_taskManager(g_taskList, sizeof(g_taskList)/sizeof(utils::CTask*), g_baseTick);

/**
 * @brief Setup function for initializing some objects and transmitting a startup message through the serial. 
 * 
 * @return uint32_t Error level codes error's type.
 */
uint32_t setup()
{
    // g_rpi.format(
    //     /* bits */ 8,
    //     /* parity */ SerialBase::None,
    //     /* stop bit */ 1
    // );
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
    uint32_t  l_errorLevel = setup(); 
    while(!l_errorLevel) 
    {
        l_errorLevel = loop();
    }
    // g_rpi.write("exiting with code: %ld",l_errorLevel, 1);
    return l_errorLevel;
}