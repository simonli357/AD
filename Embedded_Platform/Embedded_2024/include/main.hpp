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