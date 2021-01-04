# Pick-and-Place Tutorial: Part 4

This part assumes you have access to a functional ROS workspace and that the previous three parts ([Part 1](1_urdf.md), [Part 2](2_ros_tcp.md), [Part 3](2_pick_and_place.md)) have been completed.


This tutorial includes extendinig the previous tutorial so that the trajectories planned and gripper commands are executed on the simulated Niryo One as well as a Niryo One opertating in real life.

**NOTE:**
This tutorial uses the full [Niryo One ROS](https://niryo.com/docs/niryo-one/developer-tutorials/get-started-with-the-niryo-one-ros-stack/) stack runnnig on the Niryo One robot.



**Table of Contents**
  - [The Unity Side](#the-unity-side)

---

## Part 4: Sim-and-Real

We want to execute the robot commands on the simualted and real life robots simultaneously so the robot movement controlled by the C# code is goinig to be replaced with a simple subscriber that will either execute a trajectory or operate the gripper.


### Prerequisites

Niryo uses the `RobotActionCommandGoal` message to have the robot execute a variety of acctions but we are only intested in executing trajectories and tool commands.

Looking through the [Niryo One ROS](https://github.com/NiryoRobotics/niryo_one_ros) repository, the two files can be found which give the appropriate integers to use for the `cmd_type` variables in `RobotMoveActionGoal` and `ToolCommand` messages.

From `command_type.py` we need `7` to execute a trajectory and `6` to execute tool commands.

```python
JOINTS = 1
POSE = 2
POSITION = 3
RPY = 4
SHIFT_POSE = 5
TOOL = 6
EXECUTE_TRAJ = 7
POSE_QUAT = 8
SAVED_POSITION = 9
SAVED_TRAJECTORY = 10
```


From `end_effectors.yaml` we need `2` to close the gripper and `1` to open it. 

```yaml
command_list:
    # Gripper
    open_gripper: 1
    close_gripper: 2
    # Vacuump pump
    pull_air_vacuum_pump: 10
    push_air_vacuum_pump: 11
    # Tools controlled by digital I/Os
    setup_digital_io: 20
    activate_digital_io: 21
    deactivate_digital_io: 22

```



# The Unity Side

Create a new script that subscribes to the `niryo_one/commander/robot_action` topic.


TODO: It looks like the ROS side stilll needs to be hooked up, mainly the sim and real pnp,py file.