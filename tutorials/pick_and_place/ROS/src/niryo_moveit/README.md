### Prerequisites

Ensure physics solver is set to `Temporal Gauss Seidel` in [Physics Manager](https://docs.unity3d.com/Manual/class-PhysicsManager.html)

### ROS & Simulated Arm Only
--

**Terminal 1**

```
roscore &
rosparam set ROS_IP 192.168.50.149
rosparam set ROS_TCP_PORT 10000
rosparam set UNITY_IP 192.168.50.13
rosparam set UNITY_SERVER_PORT 5005
rosrun niryo_moveit server_endpoint.py
```

**Terminal 2**

`rosrun niryo_moveit mover.py`

**Terminal 3**

`roslaunch niryo_moveit demo.launch`

**Unity**

Press play in the Editor and press `Send Joint Angles` button in scene.


### Converting to Niryo One & Simulated Arm:
--

Convert the mover service to publish to the `“niryo_one/commander/robot_action` topic of a `RobotMoveCommand` message type.

**RobotMoveCommand.msg**

```
int32 cmd_type

float64[] joints
geometry_msgs/Point position
niryo_one_msgs/RPY rpy
niryo_one_msgs/ShiftPose shift
niryo_one_msgs/TrajectoryPlan Trajectory
geometry_msgs/Pose pose_quat
string  saved_position_name
int32 saved_trajectory_id
```

**TrajectoryPlan.msg**

```
moveit_msgs/RobotState trajectory_start
string group_name
moveit_msgs/RobotTrajectory trajectory
```

**ROS Side**

The current `mover.py` returns a plan with a `RobotTrajectory` and `RobotState` so it is likely that we will be able to convert our service to return the success of planning a trajectory while also publishing the trajectories to the corresponding topic.

**Unity Side**

Convert the current service call into a publisher and subscriber. Subscribe to the same topic as the real robot and move the simulated robot accordiingly.


### Creating Tutorials
--

- Be sure to update all URDFs to include gripper



### Resources Used
--

- [MoveIt](https://github.com/ros-planning/moveit)
- All of the launch and config files used in this package were copied from
[Niryo One ROS Stack](https://github.com/NiryoRobotics/niryo_one_ros) and edited to suit our reduced use case.


Questions:
---
- Why does trajectory planning fail when the target pose is about halfway up the robot? Are there constraints being enforced somewhere?


Notes:
---
Changing from Unity coords to ROS coords:

```
Unity: (x, y z)
ROS: (z, -x, y)
```


0 0.249 0

0, 0, 0.249