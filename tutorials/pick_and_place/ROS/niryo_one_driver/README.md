# Niryo One Driver

This package provides an interface to _ros\_control_ and handles the hardware control of motors.

Here's a global overview of this package :

![niryo one driver - global overview](https://niryo.com/wp-content/uploads/2017/12/niryo_one_driver_global_overview.png)

For each kind of motor, this packages contains a driver + a communication class between the driver and _ros\_control_.

2 types of motors :
* Dynamixel XL_320 motors for axis 5, 6 and some tools (grippers, vacuum pump). 
* Niryo Stepper Motors, connected to a CAN bus (you can find the firmware for the motors [here](https://github.com/NiryoRobotics/niryo_stepper))

The _ros\_interface_ class is an interface between Niryo One hardware and the ROS ecosystem. It handles specific commands (learning mode, calibration, ...) and sends some data (hardware status, connected tool, ...).
