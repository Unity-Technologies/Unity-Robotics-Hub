# Jevois camera ROS driver for Niryo One

This package contains a node (jevois\_ros\_driver) which allows you to control a Jevois camera through ROS interfaces (topics and services).

At Niryo we found that the Jevois camera is a good match for Niryo One. The image processing is done inside the camera, so we can spare a lot of CPU and RAM use on the Raspberry Pi inside the robot.

## Setup and installation

For hardware setup, all you need to do is plug the 2 USB cables from the camera to the back of the robot, on the Raspberry Pi USB ports (4 ports available), or on your own computer if you're using the simulation mode.

Make sure guvcview is installed:
```
sudo apt-get install guvcview
```

Also, to make sure the package and node are known by your ROS environment:
```
cd ~/catkin_ws
catkin_make # -j2 if on a RPi
source ~/.bashrc
```

## Run the Jevois node

First start a ROS master (roscore) or launch the Niryo One ROS stack (on Raspberry Pi or desktop simulation).

Start the Jevois node with rosrun:
```
rosrun niryo_one_camera_jevois jevois_ros_driver.py
```

The program will try to connect to the camera serial interface. On failure the program will exit with an error code. On success, a ROS service and ROS publisher will be setup. After that, if the camera is disconnected, the program will keep trying to reconnect to the serial interface (until you kill the node with CTRL+C).

Note: if the serial port used for JeVois is different from /dev/ttyACM0 on your computer, edit the SERIAL\_PORT constant in the [scripts/jevois\_ros\_driver.py](scripts/jevois_ros_driver.py) file with the correct port.

## Use the ROS interface

Check out how to use the ROS interface in the [example/](example) folder of this package.

### Load module

Use the "/niryo\_one/jevois/set\_module" service to load a module. Names of the modules are set in the [jevois\_ros\_driver.py](scripts/jevois_ros_driver.py#L75) file (at the end, in the main).

If a module is already running when you want to load a new module, the program will first stop the previous module, and then start the new one. If you send an empty string, the current module will be unloaded.

### Get serial data

To get a stream of data from the camera (the data will depend on the chosen module), simply subscribe to the "/niryo\_one/jevois/data" topic. You'll receive a string containing all the info. Then you need to parse that string.

## Jevois Modules

Here is the list of modules that are currently implemented. For each module you can find a documentation that gives you the returned data, so you can parse the strings you receive from the topic.

* QR Code: reads Qr Codes, Bar codes, etc, and gives you the decoded string + the coordinates of the code. ([documentation](http://jevois.org/moddoc/DemoQRcode/modinfo.html))
* Dice Counter: Counts points on each dice detected on the camera frames. ([documentation](http://jevois.org/moddoc/DiceCounter/modinfo.html))

Note: some modules (only those who need to start guvcview) may fail after some time on the Raspberry Pi. This issue is coming from guvcview itself. If you experience this with a Jevois module on the Pi, you can always run the Jevois node on another computer and use a ROS multi-machine setup to communicate with the rest of the Niryo One ROS stack.

## How to add a new module

This package was designed so you can add any other Jevois module by yourself ([list of all Jevois modules](http://jevois.org/doc/UserDemos.html)). To add a new module:

* Create a file in the "/modules" folder. You'll have to make your class inherit JevoisModule. Check the other existing modules for inspiration on how to write the code.
* Add the module to the "jevois\_manager" instance in the main() of "jevois\_ros\_driver.py".
