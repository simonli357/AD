Brain Layer
===============

The 'brain' namespace includes some util functionalities manage the general state of the vehicle. The vehicle has two (2) states: moving and brake (default state).

The angle has to be given in degrees, refering to the steering of the servomotor;. Positive angle means right direction, negative means left.
The movement is possible in two states: 
- Constant current consumption - giving the desired duty cycle of the PWM signal. This is the default state
- Constant speed - giving the desired speed in meters/second. In order to run with this state, the PID activation signal has to be sent (PIDA). PID tunning may 
be needed (PIDS command). 

If the PID controll is desired, it can be activated by default, and the values Kp, Ki, Kd and Tf substituted in main.cpp . 

.. doxygenclass::  brain::CRobotStateMachine
   :project: myproject
   :members: 
   :undoc-members:
   :private-members: