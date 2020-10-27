# Niryo One Commander

This package is a high-level interface built on top of Niryo One controllers and Moveit! utilities.

The robot\_action\_server provides an actionlib server. All robot commands go trough this server (from Niryo One Studio, Blockly server, Python API, etc). 

Here's what it does :
* Handles concurrent requests 
* Checks if the command can't be processed due to other factors (ex: learning mode)
* Validates parameters
* Calls required controllers and returns appropriate status and message

The Action for this server is RobotMove.action (including RobotMoveCommand.msg for the goal), and can be found in [niryo\_one\_msgs](https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_msgs) package.

The files _robot\_commander_, _arm\_moveit\_commander_, and _tool\_commander_ are used by this server to call the required controllers (tools, rpi, Moveit! + ros_control)
