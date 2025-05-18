#include <mbed.h>
#include <utils/taskmanager/taskmanager.hpp>
#include <periodics/blinker.hpp>
#include <utils/serial/serialmonitor.hpp>
#include <brain/robotstatemachine.hpp>
#include <periodics/encoderpublisher.hpp>
#include <signal/controllers/motorcontroller.hpp>
#include <hardware/encoders/quadratureencoder.hpp>


BufferedSerial          g_rpi(USBTX, USBRX);
hardware::drivers::CMotorDriverVnh g_motorVnhDriver(D3, D2, D4, -0.30, 0.30);
hardware::drivers::CSteeringMotor g_steeringDriver(D9, -23.0, 23.0);

const float g_baseTick = 0.0001; // seconds

periodics::CBlinker g_blinker(0.5 / g_baseTick, LED1);

float g_period_Encoder = 0.001;

/// Create a filter object for filtrating the noise appeared on the encoder.
signal::filter::lti::siso::CIIRFilter<float,1,2> g_encoderFilter(utils::linalg::CRowVector<float,1>({ -0.77777778}), utils::linalg::CRowVector<float,2>({0.11111111,0.11111111}));
/// Create a quadrature encoder object. It periodically measueres the rotary speed of the motor and applies the given filter. 
hardware::encoders::CQuadratureEncoderWithFilter g_quadratureEncoderTask(g_period_Encoder,hardware::encoders::CQuadratureCounter_TIM4::Instance(),2048,g_encoderFilter);
///Create an encoder publisher object to transmite the rotary speed of the dc motor to the RPi (if enabled)
periodics::CEncoderPublisher   g_encoderPublisher(0.01/g_baseTick,g_quadratureEncoderTask,g_rpi);
/// Create a spline based converter object to convert the volt signal to pwm signal for the motor driver.
signal::controllers::CConverterSpline<2,1> l_volt2pwmConverter({-0.22166,0.22166},{std::array<float,2>({0.1041568079746662,-0.08952760561569219}),std::array<float,2>({0.50805,0.0}),std::array<float,2>({0.1041568079746662,0.08952760561569219})});
/// Create a PID controller object, with the sampling time calculation equal to the one of the readings of the encoder;
signal::controllers::siso::CPidController<double> l_pidController( 0.115000,0.810000,0.000222,0.040000,g_period_Encoder);
/// Create a controller object based on the predefined PID controller, the quadrature encoder and the spline object;
signal::controllers::CMotorController g_controller(g_quadratureEncoderTask, l_pidController, &l_volt2pwmConverter, -310299.0, 310299.0);
/// Create the motion controller, which controls the robot states and the robot moves based on the transmitted command over the serial interface. 
brain::CRobotStateMachine g_robotstatemachine(g_period_Encoder, g_rpi, g_motorVnhDriver,g_steeringDriver, &g_controller);

/// Map for redirecting messages with the key and the callback functions. If the message key equals to one of the enumerated keys, than it will be applied the paired callback function.
utils::serial::SerialSubscriberMap g_serialMonitorSubscribers = {
    {"1",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSPEEDcommand)},
    {"2",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSTEERcommand)},
    {"3",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackBRAKEcommand)},
    {"4",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackACTIVPIDcommand)},
    {"5",mbed::callback(&g_encoderPublisher,&periodics::CEncoderPublisher::serialCallbackENCODERPUBcommand)},
    {"6",mbed::callback(&l_pidController,&signal::controllers::siso::CPidController<double>::serialCallbackTUNEPIDcommand)},
    {"7",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackMOVEcommand)}
};

utils::serial::SerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

static Thread blinkerThread(osPriorityLow,    1024, nullptr, "blinker");
static Thread encoderThread(osPriorityHigh,   2048, nullptr, "encoder");
void blinkerTask() {
    while (true) {
        g_blinker.run();
        ThisThread::sleep_for(500ms);
    }
}
void encoderTask() {
    while (true) {
        g_encoderPublisher.run();
        ThisThread::sleep_for(1ms);
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
    encoderThread.start(encoderTask);
    g_serialMonitor.start(5ms);
    ThisThread::sleep_for(osWaitForever);
}