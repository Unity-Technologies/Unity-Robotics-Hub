# Niryo One Modbus/TCP Server

The Modbus/TCP server is running on port 5020 by default.

It has been built on top of the [pymodbus](http://pymodbus.readthedocs.io/en/latest/index.html) library.

This enables you to make Niryo One communicate with a PLC, or another computer in the same network.

All 4 Modbus datastores are implemented : _Coil_, _Discrete Input_, _Holding Register_, _Input Register_. Each datastore has a different set of functionalities. Note that **each datastore contains a completely different set of data**.

_Discrete Input_ and _Input register_ are READ-ONLY tables. For Niryo One those have been used to keep the robot state.

_Coil_ and _Holding Register_ are READ/WRITE tables. For Niryo One those have been used to give user commands to the robot. Hence, those 2 tables do not contain the robot state, but the last given command.

Address tables start at 0.

## Coil

Each address contains a 1bit value.

READ/WRITE (the stored values correspond to the last given command, not the current robot state)

Accepted Modbus functions :
  * 0x01: READ\_COILS
  * 0x05: WRITE\_SINGLE\_COIL

This datastore can be used to set Digital I/O mode and state. Digital I/O numbers used for Modbus:
* 0 : 1A
* 1 : 1B
* 2 : 1C
* 3 : 2A
* 4 : 2B
* 5 : 2C

|Address| Description |
|-------|-------------|
| 0-5 | Digital I/O mode (Input = 1, Output = 0) |
| 100-105 | Digital I/O state (High = 1, Low = 0) |
| 200-299 | Can be used to store your own variables |

## Discrete Input

Each address contains a 1bit value.

READ-ONLY

Accepted Modbus functions :
* 0x02: READ\_DISCRETE\_INPUTS

This datastore can be used to read Digital I/O mode and state. See _Coil_ section above for digital I/O number mapping.

|Address| Description |
|-------|-------------|
| 0-5 | Digital I/O mode (Input = 1, Output = 0) |
| 100-105 | Digital I/O state (High = 1, Low = 0) |

## Holding Register

Each address contains a 16bit value.

READ/WRITE (the stored values correspond to the last given command, not the current robot state)

Accepted Modbus functions :
* 0x03: READ\_HOLDING\_REGISTERS
* 0x06: WRITE\_SINGLE\_REGISTER

|Address| Description |
|-------|-------------|
| 0-5 | Joints (mrad) |
| 10-12 | Position x,y,z (mm) |
| 13-15 | Orientation roll, pitch, yaw (mrad) |
| 100 | Send Joint Move command with stored joints |
| 101 | Send Pose Move command with stored position and orientation |
| 110 | Stop current command execution |
| 150 | Is executing command flag |
| 151 | Last command result* |
| 200-299 | Can be used to store your own variables |
| 300 | Learning Mode (On = 1, Off = 0) |
| 301 | Joystick Enabled (On = 1, Off = 0) |
| 310 | Request new calibration |
| 311 | Start auto calibration |
| 312 | Start manual calibration |
| 401 | Gripper open speed (100-1000) |
| 402 | Gripper close speed (100-1000) |
| 500 | Select tool from given id **|
| 510 | Open gripper with given id |
| 511 | Close gripper with given id |
| 512 | Pull air vacuum pump from given id |
| 513 | Push air vacuum pump from given id |
| 520 | Set /enable/ select  conveyor from a given id (conveyor id 1 = 1 , conveyor id 2 = 2)|
| 521 | Detach or disable conveyor form a given id (conveyor id 1 = 1 , conveyor id 2 = 2)|
| 522 | Control conveyor with a given id (conveyor id 1 = 1 , conveyor id 2 = 2)|
| 523 | Conevyor direction (backward = -1 , forward = 1)|
| 524 | Conveyor speed (0-100)(%)|
| 525 | Update conveyor id to the given id (conveyor id 1 = 1, conveyor id 2 = 2)|
| 526 | Stop conveyor with given id (conveyor id 1 = 1 , conveyor id 2 = 2)|

\*The "Last command result" gives you more information about the last executed command :
* 0 : no result yet
* 1 : success
* 2 : command was rejected (invalid params, ...)
* 3 : command was aborted
* 4 : command was canceled
* 5 : command had an unexpected error
* 6 : command timeout
* 7 : internal error

\*\* Select tool from id : you can find the tools ids [here](https://github.com/NiryoRobotics/niryo_one_ros/blob/master/niryo_one_tools/config/end_effectors.yaml). Send id "0" to detach current tool.

## Input Register

Each address contains a 16bit value.

READ-ONLY

Accepted Modbus functions :
* 0x04: READ\_INPUT\_REGISTERS

|Address| Description |
|-------|-------------|
| 0-5 | Joints 1-6 (mrad) |
| 10-12 | Position x,y,z (mm) |
| 13-15 | Orientation roll, pitch, yaw (mrad) |
| 200 | Selected tool ID (0 for no tool) |
| 300 | Learning Mode activated |
| 301 | Joystick enabled |
| 400 | Motors connection up (Ok = 1, Not ok = 0) |
| 401 | Calibration needed flag |
| 402 | Calibration in progress flag |
| 403 | Raspberry Pi 3 temperature |
| 404 | Raspberry Pi 3 available disk size |
| 405 | Raspberry Pi 3 ROS log size |
| 406 | Niryo One RPI image version n.1 |
| 407 | Niryo One RPI image version n.2 |
| 408 | Niryo One RPI image version n.3 |
| 409 | Hardware version (1 or 2) |
| 530 | Conveyor 1 connection state (Connected = 1 , Not connected = 0)|
| 531 | Conveyor 1 control status ( On = 0, Off = 1) |
| 532 | Conveyor 1 Speed (0-100 (%))|
| 533 | Conveyor 1 direction (Backward = -1, Forward = 1)|
| 540 | Conveyor 2 connection state (Connected = 1 , Not connected = 0)|
| 541 | Conveyor 2 control status ( On = 0, Off = 1) |
| 542 | Conveyor 2 Speed (0-100 (%))|
| 543 | Conveyor 2 direction (Backward = -1, Forward = 1)|

\*\* For more information about the conevyor, please check user Manuel on the site

## Connect to the Modbus/TCP server with Python, as a client :

You can test the Modbus/TCP server, for example from a remote computer on the same network.

```python
#!/usr/bin/env python
from pymodbus.client.sync import ModbusTcpClient  # you need to "pip install pymodbus" 

# Positive number : 0 - 32767
# Negative number : 32768 - 65535

def number_to_raw_data(val):
    if val < 0:
        val = (1 << 15) - val
    return val

def raw_data_to_number(val):
    if (val >> 15) == 1:
        val = - (val & 0x7FFF)
    return val


if __name__ == '__main__':
    address = 'insert the Modbus/TCP server IP address here'
    client = ModbusTcpClient(address, port=5020)
    client.connect()

    # Your code here
    # Check out the pymodbus documentation : http://pymodbus.readthedocs.io/en/latest/index.html

    client.close()

```
