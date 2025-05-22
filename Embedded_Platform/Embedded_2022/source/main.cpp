#include <mbed.h>
#include <utils/taskmanager/taskmanager.hpp>
#include <periodics/blinker.hpp>
#include <utils/serial/serialmonitor.hpp>
#include <brain/robotstatemachine.hpp>
#include <periodics/encoderpublisher.hpp>
#include <signal/controllers/motorcontroller.hpp>
#include <hardware/encoders/quadratureencoder.hpp>
#include <hardware/drivers/dcmotor.hpp>
#include <periodics/imu.hpp>

BufferedSerial          g_rpi(USBTX, USBRX, 115200);
hardware::drivers::CMotorDriverVnh g_motorVnhDriver(D3, D2, D4, -0.30, 0.30);
hardware::drivers::CSteeringMotor g_steeringDriver(D9, -23.0, 23.0);

const float g_baseTick = 0.0001; // seconds

periodics::CBlinker g_blinker(0.5 / g_baseTick, LED1);

float g_period_Encoder = 0.001;

signal::filter::lti::siso::CIIRFilter<float,1,2> g_encoderFilter(utils::linalg::CRowVector<float,1>({ -0.77777778}), utils::linalg::CRowVector<float,2>({0.11111111,0.11111111}));
hardware::encoders::CQuadratureEncoderWithFilter g_quadratureEncoderTask(g_period_Encoder,hardware::encoders::CQuadratureCounter_TIM2::Instance(),2048,g_encoderFilter);
periodics::CEncoderPublisher   g_encoderPublisher(0.01/g_baseTick,g_quadratureEncoderTask,g_rpi);
signal::controllers::CConverterSpline<2,1> l_volt2pwmConverter({-0.22166,0.22166},{std::array<float,2>({0.1041568079746662,-0.08952760561569219}),std::array<float,2>({0.50805,0.0}),std::array<float,2>({0.1041568079746662,0.08952760561569219})});
signal::controllers::siso::CPidController<double> l_pidController( 0.115000,0.810000,0.000222,0.040000,g_period_Encoder);
signal::controllers::CMotorController g_controller(g_quadratureEncoderTask, l_pidController, &l_volt2pwmConverter, -310299.0, 310299.0);
brain::CRobotStateMachine g_robotstatemachine(g_period_Encoder, g_rpi, g_motorVnhDriver,g_steeringDriver, &g_controller);
periodics::CImu g_imu(1, g_rpi, I2C_SDA, I2C_SCL);
/// 
/// Map for redirecting messages with the key and the callback functions. If the message key equals to one of the enumerated keys, than it will be applied the paired callback function.
utils::serial::SerialSubscriberMap g_serialMonitorSubscribers = {
    {"1",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSPEEDcommand)},
    {"2",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackSTEERcommand)},
    {"3",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackBRAKEcommand)},
    {"4",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackACTIVPIDcommand)},
    {"5",mbed::callback(&g_encoderPublisher,&periodics::CEncoderPublisher::serialCallbackENCODERPUBcommand)},
    {"6",mbed::callback(&l_pidController,&signal::controllers::siso::CPidController<double>::serialCallbackTUNEPIDcommand)},
    {"7",mbed::callback(&g_robotstatemachine,&brain::CRobotStateMachine::serialCallbackGOODcommand)},
};

utils::serial::SerialMonitor g_serialMonitor(g_rpi, g_serialMonitorSubscribers);

static Thread blinkerThread(osPriorityLow,    1024, nullptr, "blinker");
static Thread encoderThread(osPriorityHigh,   2048, nullptr, "encoder");
static Thread statemachineThread(osPriorityHigh,   2048, nullptr, "statemachine");
static Thread imuThread(osPriorityNormal, 2048, nullptr, "imu");

void blinkerTask() {
    while (true) {
        g_blinker.run();
        ThisThread::sleep_for(500ms);
    }
}
void encoderTask() {
    while (true) {
        g_quadratureEncoderTask._run();
        // g_encoderPublisher.run();
        ThisThread::sleep_for(1ms);
    }
}
void statemachineTask() {
    while (true) {
        g_robotstatemachine._run();
        ThisThread::sleep_for(50ms);
    }
}
void imuTask() {
    while (true) {
        g_imu._run();
        ThisThread::sleep_for(50ms);
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
    {
        // char resp[32];
        // g_robotstatemachine.serialCallbackACTIVPIDcommand("1", resp);
        // g_rpi.write("pid activated;;", strlen("pid activated;;"));
        // g_encoderPublisher.serialCallbackENCODERPUBcommand("1", resp);
        // g_rpi.write("encoder publisher activated;;", strlen("encoder publisher activated;;"));
    }
    sleep_manager_lock_deep_sleep();
    blinkerThread.start(blinkerTask);
    encoderThread.start(encoderTask);
    // statemachineThread.start(statemachineTask);
    imuThread.start(imuTask);
    g_serialMonitor.start(5ms);
    ThisThread::sleep_for(osWaitForever);
}