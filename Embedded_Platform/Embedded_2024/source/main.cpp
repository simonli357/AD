#include <main.hpp>

const float g_baseTick = 0.0001; // seconds

// Serial interface with the another device(like single board computer). It's an built-in class of mbed based on the UART communication, the inputs have to be transmitter and receiver pins. 
UnbufferedSerial g_rpi(USBTX, USBRX, 460800);

// task for blinking periodically the built-in led on the Nucleo board, signaling the code is uploaded on the nucleo.
periodics::CBlinker g_blinker(0.5 / g_baseTick, LED1);

// // task for sending periodically the instant current consumption of the battery
// periodics::CInstantConsumption g_instantconsumption(0.2 / g_baseTick, A2, g_rpi);

// // task for sending periodically the battery voltage, so to notice when discharging
periodics::CTotalVoltage g_totalvoltage(3.0 / g_baseTick, A1, g_rpi);

// task for sending periodically the IMU values
periodics::CImu g_imu(0.1/ g_baseTick, g_rpi, I2C_SDA, I2C_SCL);

// Task for controlling the encoder
periodics::CEncoder g_encoder(0.01/g_baseTick, g_baseTick, g_rpi, D2);

//PIN for a motor speed in ms, inferior and superior limit
drivers::CSpeedingMotor g_speedingDriver(0.1/g_baseTick,g_rpi,D3, g_encoder); //speed in cm/s

//PIN for angle in servo degrees, inferior and superior limit
drivers::CSteeringMotor g_steeringDriver(0.1 / g_baseTick, g_rpi, D4, g_imu, g_speedingDriver);

// Task responsible for configuring the vehicle's speed and steering over a specified duration.
// drivers::CVelocityControlDuration g_velocityControlDuration(0.1/g_baseTick, g_steeringDriver, g_speedingDriver);

// Create the motion controller, which controls the robot states and the robot moves based on the transmitted command over the serial interface. 
brain::CRobotStateMachine g_robotstatemachine(0.1/g_baseTick, g_rpi, g_steeringDriver, g_speedingDriver);

//Create task for the serial printer, which sends the telemetry data over the serial interface.
periodics::CSerialPrinter g_serialPrinter(0.1/g_baseTick, g_rpi);

// Map for redirecting messages with the key and the callback functions. If the message key equals to one of the enumerated keys, than it will be applied the paired callback function.
drivers::CSerialMonitor::CSerialSubscriberMap g_serialMonitorSubscribers = {
    // {"1",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSPEEDcommand)},
    // {"2",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSTEERcommand)},
    // {"3",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackBRAKEcommand)},
    // {"4",mbed::callback(&g_motorCalibration,&periodics::CTotalVoltage::SpeedMotorCalibration)},
    {"5",mbed::callback(&g_totalvoltage,&periodics::CTotalVoltage::TotalPublisherCommand)},
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
        // g_serialMonitor.timerCallback();
        // g_serialMonitor.run();
        g_serialMonitor.poll();
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