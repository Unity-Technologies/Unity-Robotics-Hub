# niryo_one_python_tcp_client
TCP client that communicates with the [TCP server](../../src/niryo_one_tcp_server) of the Niryo One
<br>**Notes:** The TCP server use the underlying [Python API](../../../niryo_one_python_api) package so the functions available
 will be similar with this package but may differ in some minor terms, please refer to the current documentation.

## Connection

Port of the server: 40001

## Linking

* If you want to make this package available everywhere on your computer, execute the following command in this folder: `pip install -e .`
* If you prefer to just link manually the package, just add this folder to PYTHONPATH environment variable before executing your script.

## Base script

```
from niryo_one_tcp_client import *

niryo_one_client = NiryoOneClient()
niryo_one_client.connect("10.10.10.10") # =< Replace by robot ip address

# YOUR CODE HERE

niryo_one_client.quit()
```

## Examples

See the [examples](examples) folder for existing scripts.

## Functions available

In this section we list all the functions available in the Python TCP API. The reader
should note that all the enums which will be mentioned in this section are registered in the 
[enums](niryo_one_tcp_client/enums.py) file.

* `connect(ip_address)`
    * ip_address: ip address of the robot
        * **String** type (`"10.10.10.10"`, ...)

    This function connect to the robot on the given ip_address.
    Return `True` on success or `False` on failure (with error printed)

### ARM

* `calibrate(mode)`
    * mode: calibrate in automatic mode or manual
        * **CalibrateMode** enum (`AUTO` / `MANUAL`)

    Calibrate robot motors according to the mode :
    * **Automatic**: moving each axis.
    * **Manual**: set the position as 'current calibration home position'.
    
    If calibration is not needed, this method will do nothing.
    
* `need_calibration()`
    Return a boolean indicating if calibration is needed or not

* `set_learning_mode(enabled)`
    * enabled: enable the learning mode on True / disable on False
        * **Boolean** (`True` / `False`)

    Activate or deactivate learning mode (= disable/enable torque on motors).

* `move_joints(j1, j2, j3, j4, j5, j6)`
    * j1 / j2 / j3 / j4 / j5 / j6: value (radian) for each joint
        * **float** type only

    Move the arm with a joint command.

* `move_pose(x, y, z, roll, pitch, yaw)`
    * x, y, z: value (meter) for the corresponding axis position
        * **float** type only
    * roll, pitch, yaw: value (radian) for the corresponding axis rotation
        * **float** type only

    Move the arm with a pose command.

* `shift_pose(axis, shift_value)`
    * axis: on which axis the position / rotation will be shifted
        * **RobotAxis** enum (`X`, `Y`, `Z`, `ROLL`, `PITCH`, `YAW`)
    * shift_value: value to shift the axis (radian or meter depending of axis, see move_pose)
        * **float** type

    Move the arm by shifting the current pose on 'axis' by 'shift_value'.

* `set_arm_max_velocity(percentage)`
    * percentage: percetage of max speed applied
        * **integer** type from 1 to 100

    Set the arm max velocity scaling factor.

* `enable_joystick(enabled)`
    * enabled: enable the joystick mode on True / disable on False
        * **Boolean** (`True` / `False`)

    Enable or disable joystick mode (control the robot with a joystick controller).

* `set_pin_mode(pin, pin_mode)`
    * pin: which pin will be changed of mode
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)
    * pin_mode: to which mode the pin should be changed
        * **PinMode** enum type (`INPUT`, `OUTPUT`)

    Set a digital I/O pin on INPUT or OUTPUT mode.

* `digital_write(pin, digital_state)`
    * pin: which pin will have it's state modifed
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)
    * digital_state: to which state the pin should be changed
        * **DigitalState** enum type (`LOW`, `HIGH`)

    Set a digital I/O pin to LOW or HIGH. Note that the pin must have been previously set as OUTPUT.
    <br><br>**Notes:** The pin must have been previously set as OUTPUT.

* `digital_read(pin)`
    * pin: from which pin we will read it's state
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)

    Returns the current pin state (0: LOW, 1: HIGH).

* `get_joints()`

    Returns an array containing the current angles for all 6 axis (in radian).

* `get_pose()`

    Returns a [PoseObject](niryo_one_tcp_client/pose_object.py)
    
* `pose_to_list(pose)`
    * pose: a [PoseObject](niryo_one_tcp_client/pose_object.py)
    Returns a python list : [pose.x, pose.y, pose.z, pose.roll, pose.pitch, pose.yaw]

* `get_hardware_status()`

    Returns a [HardwareStatusObject](niryo_one_tcp_client/hardware_status_object.py) containing useful info about the motors
    state, connection, temperature, etc. (temperature unit: °C)

* `get_learning_mode()`

    Returns a boolean that indicates whether learning mode is activated or not.

* `get_digital_io_state()`

    Returns a [DigitalPinObject](niryo_one_tcp_client/digital_pin_object.py) object containing information for all the 6\* 5V
    digital pins + 2\* 12V switches.

* `pick_from_pose(x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot)`

    * x, y, z: value (meter) for the corresponding axis position
        * **float** type only
    * roll, pitch, yaw: value (radian) for the corresponding axis rotation
        * **float** type only

    Performs a complete pick on the specified robot pose.  

* `place_from_pose(x_pos, y_pos, z_pos, roll_rot, pitch_rot, yaw_rot)`

    * x, y, z: value (meter) for the corresponding axis position
        * **float** type only
    * roll, pitch, yaw: value (radian) for the corresponding axis rotation
        * **float** type only  

    Performs a complete place on the specified robot pose.  

### TOOLS

* `change_tool(tool)`
    * tool: which tool we want to use now (or `NONE` to disable)
        * **RobotTool** enum type (`NONE`, `GRIPPER_1`, `GRIPPER_2`, `GRIPPER_3`, `ELECTROMAGNET_1`, `VACUUM_PUMP_1`)

    Change current attached tool. **Before you execute any action on a tool, you have to select it with this method.**

* `open_gripper(gripper, speed)`
    * gripper: which gripper type is used now
        * **RobotTool** enum type (`GRIPPER_1`, `GRIPPER_2`, `GRIPPER_3`)
    * speed: at which speed we want to open the gripper
        * **integer** type (between 0 and 1000, recommended : between 100 and 500)

    Open gripper at selected speed.

* `close_gripper(gripper, speed)`
    * gripper: which gripper type is used now
        * **RobotTool** enum type (`GRIPPER_1`, `GRIPPER_2`, `GRIPPER_3`)
    * speed: at which speed we want to open the gripper
        * **integer** type (between 0 and 1000, recommended : between 100 and 500)

    Close gripper at selected speed. The gripper will stop when it detects the object to grab.

* `pull_air_vacuum_pump(vacuum_pump)`
    * vacuum_pump: which vacuum pump type is used now
        * **RobotTool** enum type (`VACUUM_PUMP_1`)

    Activate vacuum pump (pick object).

* `push_air_vacuum_pump(vacuum_pump)`
    * vacuum_pump: which vacuum pump type is used now
        * **RobotTool** enum type (`VACUUM_PUMP_1`)

    Deactivate vacuum pump (place object)

* `setup_electromagnet(electromagnet, pin)`
    * electromagnet: which electromagnet type is used now
        * **RobotTool** enum type (`ELECTROMAGNET_1`)
    * pin: on which pin the electromagnet is connected
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)

    Setup electromagnet on digital I/O <pin> (set the pin mode to OUTPUT). **You need to select (change_tool) and setup the electromagnet (this method) before using it.**

* `activate_electromagnet(electromagnet, pin)`
    * electromagnet: which electromagnet type is used now
        * **RobotTool** enum type (`ELECTROMAGNET_1`)
    * pin: on which pin the electromagnet is connected
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)

    Activate electromagnet on digital I/O <pin> (pick object). This will set the pin state to HIGH.

* `deactivate_electromagnet(electromagnet, pin)`
    * electromagnet: which electromagnet type is used now
        * **RobotTool** enum type (`ELECTROMAGNET_1`)
    * pin: on which pin the electromagnet is connected
        * **RobotPin** enum type (`GPIO_1A`, `GPIO_1B`, `GPIO_1C`, `GPIO_2A`, `GPIO_2B`, `GPIO_2C`)

    Deactivate electromagnet on digital I/O <pin> (place object). This will set the pin state to LOW.

### VISION

* `get_target_pose_from_cam(workspace, height_offset, shape, color)`
    * workspace: name of the workspace which should be used
    * height_offset: offset in height between the coordinates given and the pose which will be returned
    * shape: shape of the target object
        * **Shape** enum type (`SQUARE`, `CIRCLE`, `ANY`)
    * color: color of the target object
        * **Color** enum type (`RED`, `BLUE`,`GREEN`,`ANY`)

    Return:
    * Status indicating whereas a error happened during the process
    * Boolean indicating whereas an object has been found
    * A [PoseObject](niryo_one_tcp_client/pose_object.py). It corresponds to a position of an object which respects
    shape & color parameters, in the workspace specified with an offset in height.
    * A string containing the shape of the object found
    * A string containing the color of the object found

    Return a [PoseObject](niryo_one_tcp_client/pose_object.py) which is the position of the object in the robot coordinate system.

* `get_target_pose_from_rel(workspace, height_offset, x_rel, y_rel, yaw_rel)`
    * workspace: name of the workspace which should be used
    * height_offset: offset in height between the coordinates given and the pose which will be returned
    * x_rel: Float in the range [0,1] which represent object relative position in the workspace according to X axis.
        * 0.0 : the object is on the left side of the workspace
        * 1.0 : the object is on the right side of the workspace
    * y_rel: Float in the range [0,1] which represent object relative position in the workspace according to Y axis.
        * 0.0 : the object is on the upper side of the workspace
        * 1.0 : the object is on the down side of the workspace
    * yaw_rel: Angle in radians of the object in the workspace. It is measure in the trigonometric coordinate system
  
    Return a [PoseObject](niryo_one_tcp_client/pose_object.py) in which the object can be picked with the current tool.

* `detect_object(workspace, shape, color)`
    * workspace: name of the visible workspace
    * shape: shape of the target object
        * **Shape** enum type (`SQUARE`, `CIRCLE`, `ANY`)
    * color: color of the target object
        * **Color** enum type (`RED`, `BLUE`,`GREEN`,`ANY`)

    Returns:
    * Status indicating whereas a error happened during the process
    * Boolean indicating whereas an object has been found
    * A list [x_rel,y_rel,yaw_rel] which contains the relative position of the object in the workspace
    shape & color parameters, in the workspace specified with an offset in height.
    * A string containing the shape of the object found
    * A string containing the color of the object found
    
    Detect is there is an object in the worksgpace of the shape and color given
  
* `get_img_compressed()`

   Return a compressed image which can be then uncompress with the function
   [uncompress_image](../../../niryo_one_camera/src/niryo_one_camera/image_functions.py)  

* `vision_pick(workspace, height_offset, shape, color)`

    * workspace: name of the workspace which should be used
    * height_offset: offset in height between the coordinates given and the pose which will be returned
    * shape: shape of the target object
        * **Shape** enum type (`SQUARE`, `CIRCLE`, `ANY`)
    * color: color of the target object
        * **Color** enum type (`RED`, `BLUE`,`GREEN`,`ANY`)

    Detects an object and then performs a complete pick.  

* `move_to_object(workspace, height_offset, shape, color)`

    * workspace: name of the workspace which should be used
    * height_offset: offset in height between the coordinates given and the pose which will be returned
    * shape: shape of the target object
        * **Shape** enum type (`SQUARE`, `CIRCLE`, `ANY`)
    * color: color of the target object
        * **Color** enum type (`RED`, `BLUE`,`GREEN`,`ANY`)

    Detects an object and moved directly to the corresponding position.
    
* `get_calibration_object()`

    Give intrinsics matrix and distortion vector
    Return 
    * the intrinsics matrix
    * distortion factor
    
### WORKSPACE

* `create_workspace(name, pose_origin, pose_1, pose_2, pose_3)`

    * name: name of the workspace
    * pose_origin: pose object when robot is pointing on marker 0
    * pose_1: pose object when robot is pointing on marker 1
    * pose_2: pose object when robot is pointing on marker 2
    * pose_3: pose object when robot is pointing on marker 3

    Create or update a workspace from the given poses.  

* `remove_workspace(name)`

    * name: name of the workspace to remove

    Remove a workspace.  

* `get_current_tool_id()`

    Returns the current RobotTool object

* `get_workspace_ratio(workspace_name)`

    * workspace_name: name of the workspace

    Returns the aspect ratio (width/height) of the workspace if the special marker is in the top left corner.

* `get_workspace_list()`

    Return the names of all available workspaces on the robot.


### CONVEYOR

* `set_conveyor(conveyor_id, activate)`

    * conveyor_id: id of the target conveyor
        * **ConveyorID** enum type (`ID_1`, `ID_2`)
    * activate: set the conveyor as used (equipped) on true (unequipped on false)
        * **bool** type only

    Set the target conveyor as used or not 
    
* `activate_conveyor(conveyor_id)`

    * conveyor_id: id of the target conveyor
        * **ConveyorID** enum type (`ID_1`, `ID_2`)

    Set the target conveyor as used
    
* `deactivate_conveyor(conveyor_id)`

    * conveyor_id: id of the target conveyor
        * **ConveyorID** enum type (`ID_1`, `ID_2`)

    Set the target conveyor as not used
    
* `control_conveyor(conveyor_id, control_on, speed, direction)`

    * conveyor_id: id of the target conveyor
        * **ConveyorID** enum type (`ID_1`, `ID_2`)
    * control_on: activate the conveyor if true (stop on false)
        * **bool** type only
    * speed: percentage of speed (0 -> 100)
        * **int** type only
    * direction: indicates in which direction the conveyor should run
        * **ConveyorDirection** enum type (`FORWARD`, `BACKWARD`)

    Control the target conveyor in speed + direction
    
* `stop_conveyor(conveyor_id)`

    * conveyor_id: id of the target conveyor
        * **ConveyorID** enum type (`ID_1`, `ID_2`)
    Set the target conveyor speed at 0  

* `update_conveyor_id(old_id, new_id)`

    * old_id: current conveyor id
        * **ConveyorID** enum type (`ID_1`, `ID_2`)
    * new_id: id wanted for the conveyor
        * **ConveyorID** enum type (`ID_1`, `ID_2`)

    Set the target conveyor (old_id) as new conveyor id (new_id)  
