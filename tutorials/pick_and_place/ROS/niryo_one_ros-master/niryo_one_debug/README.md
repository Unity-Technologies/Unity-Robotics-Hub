# Niryo One Debug

This package provides tools to debug parts of Niryo One.

## send\_custom\_dxl\_value.py

Use this Python script to set a motor register value (only during Niryo One runtime).

--> Useful to change RAM values at runtime.

```
cd scripts
./send_custom_dxl_value.py --help 
```

## dxl\_debug\_tools (Cpp)

Use this tool **ONLY WHEN NIRYO ONE ROS STACK IS NOT RUNNING** to scan motors, change motor id, baudrate, and any other register
 
--> Useful to setup a new Dynamixel motor (ID and baudrate), do advanced debugging, and set EEPROM values.

Make sure you have compiled the C++ file before:

```
cd ~/catkin_ws
catkin_make -j2
cd ~/catkin_ws/devel/lib/niryo_one_debug
./dxl_debug_tools --help
```

#### Example 1 (XL-320)

You have a new XL-320 motor (default ID: 1, default baudrate: 1000000 bps).

To change the ID of the motor to 12 (register address: 3, value: 12, byte size: 1): 

```
./dxl_debug_tools --id 1 --set-register 3 12 1
```

(Recommended for Niryo One) To change the shutdown register (we use value: 2 instead of the default 3)

```
./dxl_debug_tools --id 12 --set-register 18 2 1
```

#### Example 2 (XL-430)

You have a new XL-430 motor (default ID: 1, default baudrate: 57600 bps).

To change the baudrate of the motor to 1000000 bps (register address: 8, value: 3, byte size: 1)

```
./dxl_debug_tools --baudrate 57600 --id 1 --set-register 8 3 1
```

To change the ID of the motor to 19 (register address: 7, value: 19, byte size: 1)

```
./dxl_debug_tools --id 1 --set-register 7 19 1
```

## Where to find Dynamixel registers?

* [XL-320 doc](http://emanual.robotis.com/docs/en/dxl/x/xl320/#control-table-of-eeprom-area)
* [XL-430 doc](http://emanual.robotis.com/docs/en/dxl/x/xl430-w250/#control-table-of-eeprom-area)
