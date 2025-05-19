Periodics namespace
====================

The 'Periodics' namespace aims to present some usage of periodic tasks that may be used in the project. It includes 
three tasks, a blinker and an encoder publisher. The blinker has functionality to 
toggle a built-in led based on a Task and the encoder publisher is designed to send the measured rotary speed of motor to the other device
through serial port. 

.. image:: pics/diagrams/pics/Class_Periodics.png
    :align: center

.. doxygenclass::  periodics::CBlinker
   :project: myproject
   :members: 
   :undoc-members:
   :private-members:

.. doxygenclass::  periodics::CEncoderPublisher
   :project: myproject
   :members: 
   :undoc-members:
   :private-members:

