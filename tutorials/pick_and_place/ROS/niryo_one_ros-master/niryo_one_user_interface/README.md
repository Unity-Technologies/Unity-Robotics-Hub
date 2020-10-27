# Niryo One User Interface

This packages handles high-level user interface commands.

### Joystick interface

This interface uses the _joy_ ROS package to read data from a Xbox controller, and then sends commands to Niryo One joint trajectory controller. 

Each joint can be controlled separately with different buttons on the joystick.

### Sequence manager

A sequence is a set of commands that can be executed on the robot. 
For now, a sequence is created from a Blockly Xml. The Xml is then converted into some Python code which uses the Niryo One Python API.

The sequence manager allows - through a service server - to : 

* Create a sequence
* Get a sequence from ID
* Get list of sequences
* Get last executed sequence
* Update sequence
* Delete sequence

### Sequence action server

This action server is used to execute a sequence.

* Receives sequence command (from ID, from XML)
* Handles concurrent requests
* Generates Python code from XML (through a nodejs server)
* Executes the generated Python code (corresponds to the Niryo One Python API functions)
* Returns appropriate status and message

**Why a nodejs server ?**

The Blockly library from Google is running with JavaScript, so we installed a nodejs server to handle the generation of Python code from XML.

**Installation**

After you install ROS packages and execute _catkin\_make_ you still have some installation steps if you want to use Blockly on your computer or Raspberry Pi 3.

1. Make sure that you have a recent nodejs version (not the default installed one)
* curl -sL https://deb.nodesource.com/setup_8.x | sudo -E bash -
* sudo apt-get install -y nodejs
2. Install node modules in the blockly\_code\_generator directory (where you can find package.json).
* npm install
3. Create an executable
* sudo npm link
