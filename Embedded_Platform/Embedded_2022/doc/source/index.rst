Welcome to BFMC Embedded project documentation!
================================================

This documentation describes the low level application, which runs on the micro-controller Nucleo-F401RE. It aims to control the robot movement and provide an 
interface between a higher level controllers and lower level actuators and sensors. The current version uses mbed os libraries, more precisely version=5.6. The project 
has four 5 parts: brain, hardware, signal, utils and periodics. The brain folder contains the state machine of the Nucleo (speed and steering). The drivers for actuator 
and sensors are implemented in the 'hardware' folder. The 'signal' folder  includes libraries for processing signals. 'Utils' package incorporates some util functionalities. 
The periodics layer includes some periodic tasks . 

.. toctree::
   :maxdepth: 2
   :caption: Contents:
   
   development
   brain_layer
   hardware_layer
   signal_layer
   utils_layer
   periodics_layer