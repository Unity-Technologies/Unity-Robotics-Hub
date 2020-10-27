# Niyo One Raspberry Pi 3

This packages handles all the external hardware (everything else than the motors) of Niryo One, and provides many utilities for the Raspberry Pi 3 :
* Digital I/O panel : gets commands and sends the current state of digital I/Os. Also controls tools like the electromagnet.
* LED : sets the LED color.
* Top Button : executes actions when the button is pressed.
* Wifi : handles the Raspberry Pi 3 Wi-FI connection (connected to a local network, or in hotspot mode).
* ROS log : can remove all previous logs on startup to prevent a lack of disk space in the long run (SD cards do not have infinite storage).
* ROS setup : can launch different processes and roslaunch files from code, with the possibility to add a delay before launching. Practicle on Raspberry Pi, where the computation power is quite low compared to a standard laptop.

Note that this package should not be used when you are using Niryo One ROS stack on your computer, in simulation mode.
