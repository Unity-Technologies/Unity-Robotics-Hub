# Niryo One Tools

This packages is the controller for Niryo One tools. It handles tool selection and tool action. For tools controlled by Dynamixel motors, it makes sure that the tool is connected (pings the motor).

### How to develop and control your new own tool ?

We have made things easy for you to integrate a new tool. If your tool is a gripper, a vacuum pump, or a tool controlled from a digital I/O pin (2 states : ON/OFF), you'll have no code to write, just a little bit of configuration to add.

1. Make sure that the mechanical form of the tool is compatible with the robot hand (STL file [here](https://github.com/NiryoRobotics/niryo_one/tree/master/STL/7-Hand)).

2. Add a configuration for the tool

You can find all supported tools in the [config/end\_effectors.yaml](https://github.com/NiryoRobotics/niryo_one_ros/blob/master/niryo_one_tools/config/end_effectors.yaml) file. You will need to edit this file. For example, if you want to add a new gripper controlled by a Dynamixel XL\_320 motor, copy the configuration of Gripper 1 and fill it with your own values. Note that for Dynamixel tools, you need to choose a different motor ID for each tool.

3. If the motor is not a (gripper OR vacuum pump OR tool controlled from digital I/O), you'll have to write some code in _tools_ and _tool\_ros\_command\_interface_ files. However, most of the use cases for Niryo One are manipulation cases (pick and place), so the predefined set of tool types should be enough 99% of the time.
