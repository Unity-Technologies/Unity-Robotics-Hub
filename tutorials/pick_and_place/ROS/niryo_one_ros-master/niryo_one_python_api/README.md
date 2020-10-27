# Niryo One Python API

To use Python API :

1. Connect to Niryo One via ssh
    On Linux, open a terminal and type ssh niryo@(ip_address). When asked for a password, type "robotics". This is the default password, you can change it if you want.
    
    On Windows, open an ssh client with Putty. (username : "niryo", password : "robotics")
    
    you can find robot's IP by clicking on the “Search for robots in network” button on Niryo One Studio. On Linux, you can also use the nmap client to find hosts in your network.
    
    For more information, please refer to: <https://niryo.com/docs/niryo-one/developer-tutorials/connect-to-the-raspberry-pi-3-via-ssh/>

2. Create a Python file
    ```
    touch test_python_api.py
    chmod +x test_python_api.py
    ```
3. Use this template to write your code

    By writing directly in ssh on the raspberry with nano text editor
    ```
    nano test_python_api.py
    ```
   
   Follow this python template:
    ```python
    #!/usr/bin/env python
    from niryo_one_python_api.niryo_one_api import *
    import rospy
    import time
    
    rospy.init_node('niryo_one_example_python_api')
    
    n = NiryoOne()
    
    try:
        # Your code here
    except NiryoOneException as e:
        print e
        # Handle errors here
    ```
    You can find some examples [here](https://github.com/NiryoRobotics/niryo_one_ros/tree/master/niryo_one_python_api/examples).

    You can also write the python script on your own computer with your favorite text editor. Then send it to the raspberry by scp:
    ```
   scp -p SOURCE_FOLDER_PATH/FILE.py niryo@ROBOT_IP:DESTINATION_FOLDER_PATH
    ```
   Ex: This command will send test.txt file to the /home/niryo folder:
   ```
   scp -p ./test.txt niryo@169.254.200.200:
   ```
   
  4. Run your code on RPi
  
    ```
    python test_python_api.py
    ```
## Documentation

### Constants

You can use those predefined constants (instead of numbers) in the Python API methods. Please read the examples to see how to use them.

##### Tools IDs

* TOOL\_NONE
* TOOL\_GRIPPER\_1\_ID
* TOOL\_GRIPPER\_2\_ID
* TOOL\_GRIPPER\_3\_ID
* TOOL\_ELECTROMAGNET\_1\_ID
* TOOL\_VACUUM\_PUMP\_1\_ID

##### Digital pins

* PIN\_MODE\_OUTPUT
* PIN\_MODE\_INPUT
* PIN\_HIGH
* PIN\_LOW
* GPIO\_1A
* GPIO\_1B
* GPIO\_1C
* GPIO\_2A
* GPIO\_2B
* GPIO\_2C
* SW\_1
* SW\_2

##### Status

* OK
* KO

##### For shift\_pose function

* AXIS\_X
* AXIS\_Y
* AXIS\_Z
* ROT\_ROLL
* ROT\_PITCH
* ROT\_YAW

##### For Conveyor
* CONVEYOR_DIRECTION_BACKWARD
* CONVEYOR_DIRECTION_FORWARD
* CONVEYOR_ID_ONE
* CONVEYOR_ID_TWO

##### Vision Color
* COLOR_RED
* COLOR_GREEN
* COLOR_BLUE
* COLOR_ANY

##### Vision Shape
* SHAPE_CIRCLE
* SHAPE_SQUARE
* SHAPE_ANY

### Custom type

##### RobotState
    position:
        x: (float)
        y: (float)
        z: (float)
    rpy:
        roll: (float)
        pitch: (float)
        yaw: (float)
        
### Class methods

#### Calibration

##### calibrate\_auto()

Calibrate robot motors automatically (by moving axis). If calibration is not needed, this method will do nothing.

##### calibrate\_manual()

Calibrate robot motors manually (the robot just needs to be in 'home' position and to have been auto calibrated at least once). If calibration is not needed, this method will do nothing.

#### Learning mode

##### activate\_learning\_mode(set\_bool)

Parameters:
* activate (0 or 1)

Activate or deactivate learning mode (= disable/enable torque on motors).

##### get\_learning\_mode()

Returns a boolean that indicates whether learning mode is activated or not.

#### Move

##### move\_joints(joints)

Parameters:
* joints (array of 6 joints) **(rad)** in **float**

Move the arm with a joint command.

##### move\_pose(x, y, z, roll, pitch, yaw)

Parameters:
* x: value of x position **(m)** in **float**
* y: value of y position **(m)** in **float**
* z: value of z position **(m)** in **float**
* roll: value of roll rotation **(rad)** in **float**
* pitch: value of pitch rotation **(rad)** in **float**
* yaw: value of yaw rotation **(rad)** in **float**

Move the arm with a pose command.

##### shift\_pose(axis, value)

Parameters:
* axis ([Shift Pose Enum](#for-shift_pose-function))
* value (m/rad)

Move the arm by shifting the current pose on <axis> by <value>.

##### set\_arm\_max\_velocity(percentage)

Parameters:
* percentage (1-100)

Set the arm max velocity scaling factor.

##### enable\_joystick(enable)

Parameters:
* enable (0 or 1)

Enable or disable joystick mode (control the robot with a joystick controller).

#### Robot positions

##### get\_saved\_position\_list()

Get all saved positions on the robot

##### get\_joints()

Returns an array containing the current angles for all 6 axis (in radian).

##### get\_arm\_pose()

Returns a [RobotState](#robotstate) object (see in niryo\_one\_msgs package) containing the pose (position in meters + orientation in radian) of the end effector tool.

#### I/O

##### pin\_mode(pin\_id, pin\_mode)

Parameters:
* pin ([Digital pins](#digital-pins))
* mode (0: OUTPUT, 1: INPUT)

Set a digital I/O pin on INPUT or OUTPUT mode.

##### digital\_write(pin\_id, pin\_state)

Parameters:
* pin ([Digital pins](#digital-pins))
* state (0: LOW, 1: HIGH)

Set a digital I/O pin to LOW or HIGH. Note that the pin must have been previously set as OUTPUT.

##### digital\_read(pin\_id)

Parameters:
* pin ([Digital pins](#digital-pins))

Returns the current pin state (0: LOW, 1: HIGH).


##### get\_digital\_io\_state()

Returns a DigitalIOState object (see in niryo\_one\_msgs package) containing information (mode: input or output + state: high or low) for all the 6\* 5V digital pins + 2\* 12V switches.

#### End effectors

##### change\_tool(tool\_id)

Parameters:
* tool id ([Tools IDs](#tools-ids))

Change current attached tool. **Before you execute any action on a tool, you have to select it with this method.**

##### get\_current\_tool\_id()

Returns the current tool id.

##### open\_gripper(tool\_id, speed)

Parameters:
* tool id ([Tools IDs](#tools-ids))
* open speed (between 0 and 1000, recommended : between 100 and 500)

Open gripper at selected speed.

##### close\_gripper(tool\_id, speed)

Parameters:
* tool id ([Tools IDs](#tools-ids))
* close speed (between 0 and 1000, recommended : between 100 and 500)

Close gripper at selected speed. The gripper will stop when it detects the object to grab.

##### pull\_air\_vacuum\_pump(tool\_id)

Parameters:
* tool id ([Tools IDs](#tools-ids))

Activate vacuum pump (pick object).

##### push\_air\_vacuum\_pump(tool\_id)

Parameters:
* tool id ([Tools IDs](#tools-ids))

Deactivate vacuum pump (place object)

##### setup\_electromagnet(electromagnet\_id, pin)

Parameters:
* tool id ([Tools IDs](#tools-ids))
* pin ([Digital pins](#digital-pins))

Setup electromagnet on digital I/O <pin> (set the pin mode to OUTPUT). You need to select and setup the electromagnet before using it.

##### activate\_electromagnet(electromagnet\_id, pin)

Parameters:
* tool id ([Tools IDs](#tools-ids))
* pin ([Digital pins](#digital-pins))

Activate electromagnet on digital I/O <pin> (pick object). This will set the pin state to HIGH.

##### deactivate\_electromagnet(electromagnet\_id, pin)

Parameters:
* tool id ([Tools IDs](#tools-ids))
* pin ([Digital pins](#digital-pins))

Deactivate electromagnet on digital I/O <pin> (place object). This will set the pin state to LOW.

##### grab\_with\_tool(tool_id)
Parameters:
* tool id ([Tools IDs](#tools-ids))

Call grab function depending on too_id. Equivalent to close_gripper, activate_electromagnet and pull_air_vacuum_pump.

##### release\_tool(tool_id)
Parameters:
* tool id ([Tools IDs](#tools-ids))

Call release function depending on too_id. Equivalent to open_gripper, deactivate_electromagnet and push_air_vacuum_pump.

#### Others

##### get\_hardware\_status()

Returns a HardwareStatus object (see in niryo\_one\_msgs package) containing useful info about the motors state, connection, temperature, etc. (temperature unit: °C)

##### wait(time)

Parameters:
* time (seconds)

Blocks and wait for <time> seconds.

#### Workspace

##### create\_workspace(name, pose_origin, pose_1, pose_2, pose_3)
Parameters:
* name of the workspace: WORKSPACE_NAME
    * Notes: Needs to be one of the available workspaces
* robot pose when pointing at marker 0 with the pointing tool: POSE_0 (e.g. `[0.2, 0.2, 0.0, 0.0, 0.0, 0.0]` correspond to `[x, y, z, roll, pitch, yaw]` )
* robot pose when pointing at marker 1 with the pointing tool: POSE_1 (e.g. `[0.2, 0.1, 0.0, 0.0, 0.0, 0.0]`)
* robot pose when pointing at marker 2 with the pointing tool: POSE_2 (e.g. `[0.1, 0.1, 0.0, 0.0, 0.0, 0.0]`)
* robot pose when pointing at marker 3 with the pointing tool: POSE_3 (e.g. `[0.1, 0.2, 0.0, 0.0, 0.0, 0.0]`)

The marker number 0 no would be the "special" one with a black point in the center, and the others would be as follows by turning clockwise.

Creates or overwrites the workspace with the specified name. The pose arguments have to be such that **the calibration tip** touches the markers.

##### remove\_workspace(name)
Parameters:
* name of the workspace: WORKSPACE_NAME

Removes the workspace with the specified name.

##### get\_workspace\_ratio(name)
Parameters:
* name of the workspace: WORKSPACE_NAME

Returns the aspect ratio of the specified workspace. The result will be width/height assuming that the special marker is in the top left corner.

##### get\_workspace\_list()

Returns the names of all existing workspaces.

#### Compute relative pose

##### get\_target\_pose\_from\_rel(workspace, height_offset, x_rel, y_rel, yaw_rel)
Parameters:
* name of the workspace: WORKSPACE_NAME
* height offset: HEIGHT_OFFSET
    * expressed in meters
* workspace relative pose: X_REL (in [0,1]), Y_REL (in [0,1]), YAW_REL (in [-π,π])

Given a pose (x<sub>rel</sub>, y<sub>rel</sub>, yaw<sub>rel</sub>) relative to a workspace, this function returns 
the robot pose in which the current tool will be able to pick an object at this pose. 
The height_offset argument (in m) defines how high the tool will hover over the workspace. If height_offset = 0, the tool will nearly touch the workspace.
Returns: [target_pose](#robotstate)

*Make sure to call `change_tool` before!*

##### get\_target\_pose\_from\_cam(workspace, height_offset, shape, color)
Parameters:
* name of the workspace: WORKSPACE_NAME
* height offset: HEIGHT_OFFSET
    * expressed in meters
* object shape to detect: SHAPE
    * must be one of: [Shape Enum](#vision-shape)
* object color to detect: COLOR
    * must be one of: [Color Enum](#vision-color)

First detects the specified object using the camera and then returns the robot pose in which the object can 
be picked with the current tool. Returns: [object found] (bool), [object pose](#robotstate), [object shape](#vision-shape), [object color](#vision-color)

*Make sure to call `change_tool` before!*

#### Vision Pick and place functions

##### vision\_pick(workspace, height_offset, shape, color)
Parameters:
* name of the workspace: WORKSPACE_NAME
* height offset for picking: HEIGHT_OFFSET
    * expressed in m 
* object shape to detect: SHAPE
    * must be one of: [Shape Enum](#vision-shape)
* object color to detect: COLOR
    * must be one of: [Color Enum](#vision-color)

Picks the specified object from the workspace. This function has multiple phases:
1. detect object using the camera
2. prepare the current tool for picking
3. approach the object
4. move down to the correct picking pose
5. actuate the current tool
6. lift the object

Please ensure that the camera is parallel to the workspace and that the 4 markers are in the field of view.

Returns: [object found] (bool), [object shape](#vision-shape), [object color](#vision-color)
*Make sure to call `change_tool` before!*

##### move\_to\_object(workspace, height_offset, shape, color)
Same as `get_target_pose_from_cam` but directly moves to this position. Returns: [object found] (bool), [object shape](#vision-shape), [object color](#vision-color)

*Make sure to call `change_tool` before!*

##### pick\_from\_pose(x, y, z, roll, pitch, yaw)
Parameters:
* x: value of x position **(m)** in **float**
* y: value of y position **(m)** in **float**
* z: value of z position **(m)** in **float**
* roll: value of roll rotation **(rad)** in **float**
* pitch: value of pitch rotation **(rad)** in **float**
* yaw: value of yaw rotation **(rad)** in **float**

Pick at the specified robot pose. This function has multiple phases:
1. move the specified pose with an offset in z
2. move down to the specified pose
3. actuate (close) the current tool
4. lift the object

*Make sure to call `change_tool` before!*

##### place\_from\_pose(x, y, z, roll, pitch, yaw)
Parameters:
* x: value of x position **(m)** in **float**
* y: value of y position **(m)** in **float**
* z: value of z position **(m)** in **float**
* roll: value of roll rotation **(rad)** in **float**
* pitch: value of pitch rotation **(rad)** in **float**
* yaw: value of yaw rotation **(rad)** in **float**

Place at the specified robot pose. This function has multiple phases:
1. move the specified pose with an offset in z
2. move down to the specified pose
3. actuate (open) the current tool
4. move up again

*Make sure to call `change_tool` before!*

##### detect\_object( workspace, shape, color):
Parameters:
* name of the workspace: WORKSPACE_NAME
* object shape to detect: SHAPE
    * must be one of: [Shape Enum](#vision-shape)
* object color to detect: COLOR
    * must be one of: [Color Enum](#vision-color)

Please ensure that the camera is parallel to the workspace and that the 4 markers are in the field of view.

Detects the specified object using the camera. Returns [object found] (bool), [relative object pose](#robotstate), [object shape](#vision-shape), [object color](#vision-color)

#### Camera image

##### get\_compressed\_image()
Returns the most recent camera image in compressed format. 
Uncompress e.g. using the following code snippet:
```python
import numpy as np
import cv2


def f(compressed_image):
    np_arr = np.fromstring(compressed_image, np.uint8)
    img = cv2.imdecode(np_arr, cv2.IMREAD_COLOR)
    return img
```

##### get\_calibration\_object()
Return intrinsics matrix, distortion coefficients, new intrinsics matrix

#### Conveyor

##### set\_conveyor(conveyor_id, activate):

Parameters:
* conveyor_id (int) id of the conveyor ([Conveyor Enum](#for-conveyor))
* activate (bool) equip / unequip the conveyor

Set the given conveyor (ID) as being equipped/not unequipped (similar to change_tool)

##### control\_conveyor(conveyor_id, control_on, speed, direction)

Parameters:
* conveyor_id (int) id of the conveyor ([Conveyor Enum](#for-conveyor))
* control_on (bool) on/off conveyor 
* speed (int) speed as percentage
* direction (int) ([Conveyor Enum](#for-conveyor))

Control the given conveyor with enabled + speed + direction

##### update\_conveyor_id(old_id, new_id)

Parameters:
* old\_id (int) id of the current conveyor_id ([Conveyor Enum](#for-conveyor))
* new\_id (int) id to set as current conveyor_id ([Conveyor Enum](#for-conveyor))

Update the current conveyor id (old_id) to the conveyor id wanted (new_id)

##### get\_conveyor\_1\_feedback()
Return the current conveyor\_1 feedback (id / is\_equipped / is\_running / speed / direction)

##### get\_conveyor\_2\_feedback()
Return the current conveyor\_2 feedback (id / is\_equipped / is\_running / speed / direction)

#### Some useful functions

##### list\_to\_robot\_state\_msg(list\_pos)
Parameters:
* list\_pos (float) which represents a pose as following
```[x, y, z, roll, pitch, yaw]```

Return [RobotState](#robotstate) conversion

##### robot\_state\_msg\_to\_list(robot\_state):
Parameters:
* robot\_states: [RobotState](#robotstate)

Return a float array conversion that represents the pose as following
```[x, y, z, roll, pitch, yaw]```
