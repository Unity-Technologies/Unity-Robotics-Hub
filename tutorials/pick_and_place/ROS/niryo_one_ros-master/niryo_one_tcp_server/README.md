# Niryo One Tcp Server
This a TCP server built on top of the [Niryo Python API](../niryo_one_python_api).

<br>Programs can communicate through network TCP with the robots in any language available. (see [clients](clients) folder for available clients)

It offers a simple way for developers to create programs for robot to control them via remote communication on a computer, on a mobile or any device with network facilities.

## Documentation

### Architecture

![Niryo One TCP architecture](img/Niryo_One_TCP_Architecture.jpg)

### Connection

Port of the server: 40001

### Communication

* Only one client can communicate with the server (reconnection effective but no multi clients).
* The server answer only after the command is done, so it cannot deal with multiple commands at the same time.

### Format

For easier usage and easier debugging, the communication is in ASCII format.

#### General format

* For command without parameter, the format is:
    * `COMMAND`
* For command with parameter, the format is:
    * `COMMAND:param1,param2`

#### Answer format

* For all commands, the result will looks like this:
    * `COMMAND:OK`
        * Example: `OPEN_GRIPPER:OK`
    * `COMMAND:OK,DATA`
        * Example: `DIGITAL_READ:OK,1`
    * `COMMAND:KO,"REASON"`
        * Given: `SET_LEARNING_MODE`
        * Returned error: `SET_LEARNING_MODE:KO,"Incorrect number of parameter(s) given."`

**Notes:** That means, if you develop you own tcp client, just parse the status ('OK' / 'KO') after your given command and act in consequence.

### Common error answers

* Expected one of the following: `expected_choice`. Given: `value`.
* Expected the following type: `expected_type`. Given: `type`.
* Expected: `expected_nbr` parameters, given: `number_parameters`.

### Clients

For already existing clients, please go to the [clients](clients) folder and check if there's an already existing client that communicate with the Niryo One across TCP/IP.

### Commands

* "CALIBRATE"
* "SET_LEARNING_MODE"
* "MOVE_JOINTS"
* "MOVE_POSE"
* "SHIFT_POSE"
* "SET_ARM_MAX_VELOCITY"
* "ENABLE_JOYSTICK"
* "SET_PIN_MODE"
* "DIGITAL_WRITE"
* "DIGITAL_READ"
* "CHANGE_TOOL"
* "OPEN_GRIPPER"
* "CLOSE_GRIPPER"
* "PULL_AIR_VACUUM_PUMP"
* "PUSH_AIR_VACUUM_PUMP"
* "SETUP_ELECTROMAGNET"
* "ACTIVATE_ELECTROMAGNET"
* "DEACTIVATE_ELECTROMAGNET"
* "GET_JOINTS"
* "GET_POSE"
* "GET_HARDWARE_STATUS"
* "GET_LEARNING_MODE"
* "GET_DIGITAL_IO_STATE"
* "GET_DIGITAL_IO_STATE"
* "GET_IMAGE_COMPRESSED"
* "CREATE_WORKSPACE"
* "REMOVE_WORKSPACE"
* "GET_TARGET_POSE_FROM_REL"
* "GET_TARGET_POSE_FROM_CAM"
* "DETECT_OBJECT"
* "GET_CURRENT_TOOL_ID"
* "GET_WORKSPACE_RATIO"
* "GET_WORKSPACE_LIST"
* "VISION_PICK"
* "MOVE_TO_OBJECT"
* "PICK_FROM_POSE"
* "PLACE_FROM_POSE"
* "SET_CONVEYOR"
* "CONTROL_CONVEYOR"
* "UPDATE_CONVEYOR_ID"
* "GET_CALIBRATION_OBJECT"

#### CALIBRATE

Parameters:
* Mode:
    * AUTO for an automatic calibration
    * MANUAL for a manual calibration

**Example:** `CALIBRATE:AUTO`
<br>**Answers:** See [answer format section](#answer-format)

#### SET_LEARNING_MODE

Parameters:
* enabled:
    * TRUE to activate learning mode
    * FALSE to deactivate learning mode

**Example:**
    `SET_LEARNING_MODE:TRUE`
<br>**Answers:** See [answer format section](#answer-format)

#### MOVE_JOINTS

Parameters:
* j1: value of joints **(rad)** in **float**
* j2: value of joints **(rad)** in **float**
* j3: value of joints **(rad)** in **float**
* j4: value of joints **(rad)** in **float**
* j5: value of joints **(rad)** in **float**
* j6: value of joints **(rad)** in **float**

**Example:**
    `MOVE_JOINTS:0.03,0.0123,0.456,0.987,0.654,0.321`
<br>**Answers:** See [answer format section](#answer-format)

#### MOVE_POSE

Parameters:
* x: value of x position **(m)** in **float**
* y: value of y position **(m)** in **float**
* z: value of z position **(m)** in **float**
* roll: value of roll rotation **(rad)** in **float**
* pitch: value of pitch rotation **(rad)** in **float**
* yaw: value of yaw rotation **(rad)** in **float**

**Example:**
    `MOVE_JOINTS:0.03,0.0123,0.456,0.987,0.654,0.321`
<br>**Answers:** See [answer format section](#answer-format)

#### SHIFT_POSE

Parameters:
* axis: X / Y / Z / ROLL / PITCH / YAW
* value: value to shift the desired axis in **float**
    * Notes: Same units as before (m / rad) depending of axis

**Example:**
    `SHIFT_POSE:ROLL,0.03142`
<br>**Answers:** See [answer format section](#answer-format)

#### SET_ARM_MAX_VELOCITY

Parameters:
* percentage: percentage of max velocity in **integer**

**Example:**
    `SET_ARM_MAX_VELOCITY:50`
<br>**Answers:** See [answer format section](#answer-format)

#### ENABLE_JOYSTICK

Parameters:
* enabled: TRUE / FALSE

**Example:**
    `ENABLE_JOYSTICK:FALSE`
<br>**Answers:** See [answer format section](#answer-format)

#### SET_PIN_MODE

Parameters:
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C
* pin_mode: OUTPUT / INPUT

**Example:**
    `SET_PIN_MODE:GPIO_2B,OUTPUT`
<br>**Answers:** See [answer format section](#answer-format)

#### DIGITAL_WRITE

Parameters:
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C
* pin_state: LOW / HIGH

**Example:**
    `DIGITAL_WRITE:GPIO_2A,LOW`
<br>**Answers:** See [answer format section](#answer-format)

**Notes:** The pin must have been previously set as OUTPUT.

#### DIGITAL_READ

Parameters:
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

**Example:**
    `DIGITAL_READ:GPIO_1A`
<br>**Answers:** See [answer format section](#answer-format)

#### CHANGE_TOOL

Parameters:
* tool: GRIPPER_1 / GRIPPER_2 / GRIPPER_3 / VACUUM_PUMP_1 / ELECTROMAGNET_1

**Example:**
    `CHANGE_TOOL:GRIPPER_2`
<br>**Answers:** See [answer format section](#answer-format)

**Notes:** Before you execute any action on a tool, you have to select it with this method.

#### OPEN_GRIPPER

Parameters:
* gripper_type: GRIPPER_1 / GRIPPER_2 / GRIPPER_3
* speed: speed value as **integer**
    * **Between** 0 and 1000.
    * **Recommended** : between 100 and 500

**Example:**
    `OPEN_GRIPPER:GRIPPER_3,200`
<br>**Answers:** See [answer format section](#answer-format)

#### CLOSE_GRIPPER

Parameters:
* gripper_type: GRIPPER_1 / GRIPPER_2 / GRIPPER_3
* speed: speed value as **integer**
    * **Between** 0 and 1000.
    * **Recommended** : between 100 and 500

**Example:**
    `CLOSE_GRIPPER:GRIPPER_1,200`
<br>**Answers:** See [answer format section](#answer-format)

#### PULL_AIR_VACUUM_PUMP

Parameters:
* vacuum_type: VACUUM_PUMP_1
    * Notes: Only one type available for now

**Example:**
    `PULL_AIR_VACUUM_PUMP:VACUUM_PUMP_1`
<br>**Answers:** See [answer format section](#answer-format)

#### PUSH_AIR_VACUUM_PUMP

Parameters:
* vacuum_type: VACUUM_PUMP_1
    * Notes: Only one type available for now

**Example:**
    `PUSH_AIR_VACUUM_PUMP:VACUUM_PUMP_1`
<br>**Answers:** See [answer format section](#answer-format)

#### SETUP_ELECTROMAGNET

Parameters:
* electromagnet_type: ELECTROMAGNET_1
    * Notes: Only one type available for now
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

**Example:**
    `SETUP_ELECTROMAGNET:ELECTROMAGNET_1,GPIO_2B`
<br>**Answers:** See [answer format section](#answer-format)

#### ACTIVATE_ELECTROMAGNET

Parameters:
* electromagnet_type: ELECTROMAGNET_1
    * Notes: Only one type available for now
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

**Example:**
    `ACTIVATE_ELECTROMAGNET:ELECTROMAGNET_1,GPIO_2B`
<br>**Answers:** See [answer format section](#answer-format)

#### DEACTIVATE_ELECTROMAGNET

Parameters:
* electromagnet_type: ELECTROMAGNET_1
    * Notes: Only one type available for now
* pin: GPIO_1A / GPIO_1B / GPIO_1C / GPIO_2A / GPIO_2B / GPIO_2C

**Example:**
    `DEACTIVATE_ELECTROMAGNET:ELECTROMAGNET_1,GPIO_2C`
<br>**Answers:** See [answer format section](#answer-format)

#### GET_JOINTS

No parameters

**Example:**
    `GET_JOINTS`
<br>**Answers:**
* On success the format is `GET_JOINTS:OK,j1,j2,j3,j4,j5,j6`:
    * ex: `GET_JOINTS:OK,0.0,0.640187,-1.397485,0.0,0.0,0.0`
    * j1, ..., j6 values are expressed in radian

#### GET_POSE

No parameters

**Example:**
    `GET_POSE`
<br>**Answers:**
* On success the format is `GET_POSE:OK,x,y,z,roll,pitch,yaw
    * ex: `GET_POSE:OK,0.0695735635306,1.31094787803e-12,0.200777981243,-5.10302119597e-12,0.757298,5.10351727471e-12`
    * x, y, z values are expressed in meter
    * roll, pitch, yaw are expressed in radian

#### GET_HARDWARE_STATUS

No parameters

**Example:**
    `GET_HARDWARE_STATUS`
<br>**Answers:**
* On success the format is `GET_HARDWARE_STATUS:OK,rpi_temperature,hardware_version,connection_up,error_message,calibration_needed,calibration_in_progress,[motor_names],[motor_types],[temperatures],[voltages],[hardware_errors]`
    * ex: `GET_HARDWARE_STATUS:OK,59,2,True,'',0,False,['Stepper Axis 1', 'Stepper Axis 2', 'Stepper Axis 3', 'Servo Axis 4', 'Servo Axis 5', 'Servo Axis 6'],['Niryo Stepper', 'Niryo Stepper', 'Niryo Stepper', 'DXL XL-430', 'DXL XL-430', 'DXL XL-320'],(34, 34, 37, 43, 45, 37),(0.0, 0.0, 0.0, 11.3, 11.2, 7.9),(0, 0, 0, 0, 0, 0)`
#### GET_LEARNING_MODE

No parameters

**Example:**
    `GET_LEARNING_MODE`
<br>**Answers:** See [answer format section](#answer-format)

#### GET_DIGITAL_IO_STATE

No parameter

**Example:**
    `GET_DIGITAL_IO_STATE`
<br>**Answers:**
* On success the format is `GET_DIGITAL_IO_STATE:OK,[pin_id, name, mode, state],...` ('...' means that the `[pin_id, name, mode, state]` block is repeated for each pin)
    * ex: `GET_DIGITAL_IO_STATE:OK,[2, '1A', 1, 1],[3, '1B', 1, 1],[16, '1C', 1, 1],[26, '2A', 1, 1],[19, '2B', 1, 1],[6, '2C', 1, 1],[12, 'SW1', 0, 0],[13, 'SW2', 0, 0]`

#### GET_IMAGE_COMPRESSED

No parameters

**IMPORTANT** Due to the large and variable image size, a new answer format is needed, see the examples below. The client is expected to first receive the Command:OK and the image size. Then it should continue receiving until the size of the received image equals image size.
**Example:**
    `GET_IMAGE_COMPRESSED`
<br>**Answers:**
* On success the format is `GET_IMAGE_COMPRESSED:OK,IMAGE_SIZE,IMAGE`
* ex: `GET_IMAGE_COMPRESSED:OK,12345,...`

#### CREATE_WORKSPACE

Parameters:
* name of the workspace: WORKSPACE_NAME
    * Notes: Needs to be one of the available workspaces
* robot pose when pointing at marker 0: POSE_0 (e.g. `[0.2, 0.2, 0.0, 0.0, 0.0, 0.0]`)
* robot pose when pointing at marker 1: POSE_1 (e.g. `[0.2, 0.1, 0.0, 0.0, 0.0, 0.0]`)
* robot pose when pointing at marker 2: POSE_2 (e.g. `[0.1, 0.1, 0.0, 0.0, 0.0, 0.0]`)
* robot pose when pointing at marker 3: POSE_3 (e.g. `[0.1, 0.2, 0.0, 0.0, 0.0, 0.0]`)

**Example:**
    `CREATE_WORKSPACE,WORKSPACE_NAME,[0.2, 0.2, 0.0, 0.0, 0.0, 0.0, 0.2, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.1, 0.0, 0.0, 0.0, 0.0, 0.1, 0.2, 0.0, 0.0, 0.0, 0.0]`
<br>**Answers:** See [answer format section](#answer-format)

#### REMOVE_WORKSPACE
Parameters:
* name of the workspace: WORKSPACE_NAME

**Example:**
    `REMOVE_WORKSPACE,WORKSPACE_NAME`
<br>**Answers:** See [answer format section](#answer-format)

#### GET_TARGET_POSE_FROM_REL
Parameters:
* name of the workspace: WORKSPACE_NAME
* height offset: HEIGHT_OFFSET
    * expressed in meters
* workspace relative pose: X_REL (in [0,1]), Y_REL (in [0,1]), YAW_REL (in [-π,π])

**Example:**
    `GET_TARGET_POSE_FROM_REL,WORKSPACE_NAME,0.1,0.2,0.753`
<br>**Answers:**
* On success the format is `GET_TARGET_POSE_FROM_REL:OK,x,y,z,roll,pitch,yaw
    * ex: `GET_TARGET_POSE_FROM_REL:OK,0.0695735635306,1.31094787803e-12,0.200777981243,-5.10302119597e-12,0.757298,5.10351727471e-12`
    * x, y, z values are expressed in meters
    * roll, pitch, yaw are expressed in radians

#### GET_TARGET_POSE_FROM_CAM

Parameters:
* name of the workspace: WORKSPACE_NAME
* height offset: HEIGHT_OFFSET
    * expressed in meters
* object shape to detect: SHAPE
    * must be one of: SQUARE CIRCLE ANY
* object color to detect: COLOR
    * must be one of: RED BLUE GREEN ANY

**Example:**
    `GET_TARGET_POSE_FROM_CAM,WORKSPACE_NAME,0.02,ANY,RED`
<br>**Answers:**
* On success the format is `GET_TARGET_POSE_FROM_CAM:OK,OBJECT_FOUND,x,y,z,roll,pitch,yaw,SHAPE,COLOR`
    * where SHAPE, COLOR are the same as the input unless the input is ANY
* ex: `GET_TARGET_POSE_FROM_CAM:OK,True,,0.0695735635306,1.31094787803e-12,0.200777981243,-5.10302119597e-12,0.757298,5.10351727471e-12,CIRCLE,RED`

#### DETECT_OBJECT
Parameters:
* name of the workspace: WORKSPACE_NAME
* object shape to detect: SHAPE
    * must be one of: SQUARE CIRCLE ANY
* object color to detect: COLOR
    * must be one of: RED BLUE GREEN ANY

**Example:**
    `DETECT_OBJECT,WORKSPACE_NAME,CIRCLE,ANY`
<br>**Answers:**
* On success the format is `DETECT_OBJECT:OK,OBJECT_FOUND,X_REL,Y_REL,YAW_REL,SHAPE,COLOR`
    * where SHAPE, COLOR are the same as the input unless the input is ANY
* ex: `DETECT_OBJECT:OK,True,0.2421, 0.121, 0.731,CIRCLE,RED`

#### GET_CURRENT_TOOL_ID

No parameter

**Example:**
    `GET_CURRENT_TOOL_ID`
<br>**Answers:**
* On success the format is `GET_CURRENT_TOOL_ID:OK,TOOL_NAME`
* ex: `GET_CURRENT_TOOL_ID:OK,GRIPPER_1`

#### GET_WORKSPACE_RATIO
Parameters:
* name of the workspace: WORKSPACE_NAME

**Example:**
    `GET_WORKSPACE_RATIO,WORKSPACE_NAME`
<br>**Answers:**
* On success the format is `GET_WORKSPACE_RATIO:OK,RATIO`
* ex: `GET_WORKSPACE_RATIO:OK,0.937263`

#### GET_WORKSPACE_LIST

No parameter

**Example:**
    `GET_WORKSPACE_LIST`
<br>**Answers:**
* On success the format is `GET_WORKSPACE_LIST:OK,list_size,[workspaces]`
* ex: `GET_WORKSPACE_LIST:OK,21,Conveyor,Large_1,Wall`

#### VISION_PICK

Parameters:
* name of the workspace: WORKSPACE_NAME
* height offset: HEIGHT_OFFSET
    * expressed in m
* object shape to detect: SHAPE
    * must be one of: SQUARE CIRCLE ANY
* object color to detect: COLOR
    * must be one of: RED BLUE GREEN ANY

**Example:**
    `VISION_PICK,WORKSPACE_NAME,HEIGHT_OFFSET,CIRCLE,ANY`
<br>>**Answers:**
* On success the format is `VISION_PICK:OK,OBJECT_PICKED,SHAPE,COLOR`
* ex: `VISION_PICK:OK,True,CIRCLE,GREEN`

#### MOVE_TO_OBJECT

Parameters:
* name of the workspace: WORKSPACE_NAME
* height offset: HEIGHT_OFFSET
    * expressed in m
* object shape to detect: SHAPE
    * must be one of: SQUARE CIRCLE ANY
* object color to detect: COLOR
    * must be one of: RED BLUE GREEN ANY

**Example:**
    `MOVE_TO_OBJECT,WORKSPACE_NAME,HEIGHT_OFFSET,CIRCLE,ANY`
<br>**Answers:**
* On success the format is `MOVE_TO_OBJECT:OK,OBJECT_FOUND,SHAPE,COLOR`
* ex: `MOVE_TO_OBJECT:OK,True,CIRCLE,GREEN`

#### PICK_FROM_POSE

Parameters:
* x: value of x position **(m)** in **float**
* y: value of y position **(m)** in **float**
* z: value of z position **(m)** in **float**
* roll: value of roll rotation **(rad)** in **float**
* pitch: value of pitch rotation **(rad)** in **float**
* yaw: value of yaw rotation **(rad)** in **float**

**Example:**
    `PICK_FROM_POSE:0.03,0.0123,0.456,0.987,0.654,0.321`
<br>**Answers:** See [answer format section](#answer-format)

#### PLACE_FROM_POSE

Parameters:
* x: value of x position **(m)** in **float**
* y: value of y position **(m)** in **float**
* z: value of z position **(m)** in **float**
* roll: value of roll rotation **(rad)** in **float**
* pitch: value of pitch rotation **(rad)** in **float**
* yaw: value of yaw rotation **(rad)** in **float**

**Example:**
    `PLACE_FROM_POSE:0.03,0.0123,0.456,0.987,0.654,0.321`
<br>**Answers:** See [answer format section](#answer-format)

#### SET_CONVEYOR

Parameters:
* conveyor_id: id of the conveyor 6 (conveyor 1) or 7 (conveyor 2)
* equipped: TRUE to use the conveyor / FALSE to unused

**Example:**
    `SET_CONVEYOR:6,TRUE`
<br>**Answers:** See [answer format section](#answer-format)

#### CONTROL_CONVEYOR

Parameters:
* conveyor_id: id of the conveyor 6 (conveyor 1) or 7 (conveyor 2)
* control_on: TRUE to activate the conveyor / FALSE to stop the conveyor
* speed: percentage of speed wanted (between 0 and 100)
* direction: indicates in which direction the conveyor should run: -1 (BACKWARD) or 1 (FORWARD)

**Example:**
    `CONTROL_CONVEYOR:6,TRUE,80,1`
<br>**Answers:** See [answer format section](#answer-format)

#### UPDATE_CONVEYOR_ID

Parameters:
* current_conveyor_id: current conveyor id: 6 (conveyor 1) or 7 (conveyor 2)
* new_conveyor_id: id wanted for the conveyor: 6 (conveyor 1) or 7 (conveyor 2)

**Example:**
    `UPDATE_CONVEYOR_ID:6,7`
<br>**Answers:** See [answer format section](#answer-format)

#### GET_CALIBRATION_OBJECT

No parameters

**Example:**
    `GET_CALIBRATION_OBJECT`
<br>**Answers:**
* On success the format is `GET_CALIBRATION_OBJECT:OK,tuple_mtx,tuple_dist
    * ex: `GET_CALIBRATION_OBJECT:OK,(450,0,320,0,400,240,0,0,1),(20.1,1.5674,0.498,0.249)`
    * tuple_mtx is a flatten 3x3 matrix which represent camera intrinsics parameters
    * tuple_dist is a vector which can have different length, representing distortion factors 
