# Pick-and-Place Tutorial: Part 4

This part is going to be a little different than the previous tutorials in that it will utilize a real Niryo One robot. Although we assume you have completed the previous three parts ([Part 1](1_urdf.md), [Part 2](2_ros_tcp.md), [Part 3](2_pick_and_place.md)), we do not assume that you have access to a Niryo One outside of simulation. As such this tutorial should mostly be used as a reference for how to simulate the behaviour of a robot operating in real life.

For this tutorial we will be using the full [Niryo One ROS](https://niryo.com/docs/niryo-one/developer-tutorials/get-started-with-the-niryo-one-ros-stack/) stack running on the Niryo One robot.

From the Niryo One ROS documentation:
> The commander package exposes an action server named “niryo_one/commander/robot_action”, with a “RobotMoveCommand” message. This is probably what you’re looking for if you want to send direct command to the robot using ROS.

Commands can be executed on the Niryo One by publishing `RobotMoveActionGoal` messages directly to the `niryo_one/commander/robot_action/goal` topic, however the default behaviour of the action server only allows one action to be executed at a time. Furthermore, any new actions received by the action server will replace any currently executing actions.

For our pick and place task we want the robot to execute achieve multiple target poses so instead of publishing directly to the `goal` topic we will be usinig an action client.

**Table of Contents**

  - [Differences From Part 3](#)
  - [Finding Correct Message Values]()
  - [The ROS Side](#the-ros-side)
  - [The Unity Side](#the-unity-side)


## Differences From Part 3
**Part 3 Message Flow:**

**TODO: Diagrams**

- `TrajectoryPlanner` sends ServiceMessage with robot pose, target cube coordinates, and target destination coordinates to `mover.py`
- `mover.py` calculates the four trajectories, adds them to an array, and returns the array to `TrajectoryPlanner`
- `TrajectoryPlanner` then executes the trajectories one at a time

**The New Flow:**

- `RealSimPickAndPlace.cs` publishes robot pose, target cube coordinates, and target destination coordinates to `sim_real_pnp` topic
- `sim_and_real_pnp.py,`as a ROS subscriber node, reads from the `sim_real_pnp` topic and:

- **Simultaneously**
	- Simulated Niryo One, `RealSimPickAndPlace.cs`, subscriber reads from `robot_action/goal` topic and executes the trajectory
	- Real Niryo One reads from `robot_action/goal`  topic and executes trajectory


## Finding Correct RobotMoveGoal Values

The `RobotMoveGoal` message contains a single parameter, `RobotMoveCommand`, which is a catch all for executing a variety of commands on the robot. 


### RobotMoveCommand
Since we are only interested in executing trajectory and gripper commands we will only focus on the following parameters:

- `int32 cmd_type`
	- Which type of command to execute
- `niryo_one_msgs/TrajectoryPlan Trajectory`
	- A trajectory to be executed
- `niryo_one_msgs/ToolCommand tool_cmd`
	- The end effector command to be executed

To find the appropriate values for the `cmd_type` variable we need to look at the `niryo_one_ros/niryo_one_commander/src/niryo_one_commander/command_type.py` file:

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

From here we can see that the `cmd_type` variable in `RobotMoveCommand` will need to be `7` to execute a trajectory and `6` to execute a tool command.

### ToolCommand

Much like the `RobotMoveGoal`, the `ToolCommand` message type is a catch all for gripper commands since the Niryo One supports a variety of [grippers](https://niryo.com/niryo-one-accessories/).
We are only interested in opening and closing the standard gripper so we will only use the following parameters:

- `uint8 tool_id`
	- Which end effector is being used
- `uint8 cmd_type`
	- Which command to execute. Ex. open or close
- `uint16 gripper_close_speed`
	- How fast should the gripper close
- `uint16 gripper_open_speed`
	- How fast should the gripper close

Just like the `cmd_type` parameter in `RobotMoveCommand` we will need to look at `niryo_one_ros/niryo_one_tools/config/end_effectors.yaml` to find the appropriate values to us.

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

From here we see that the `cmd_type` variable in `ToolCommand` will need to be `2` to close the gripper and `1` to open it. 


# The ROS Side
---
There are two lines that have not been previously used in `server_endpoint.py` that we will now be using.

- The following line will be used by the simulated Niryo One in our Unity scene to read from the `robot_action/goal` topic and execute the same trajectories as the real Niryo One.
```python
'niryo_one/commander/robot_action/goal': RosSubscriber('niryo_one/commander/robot_action/goal', RobotMoveActionGoal, tcp_server),
```

- We will be reusing the `MoverServiceRequest` message type to publish the target and target destination poses to ROS from Unity.
```python
'sim_real_pnp': RosPublisher('sim_real_pnp', MoverServiceRequest)
```

We will also make use of the `sim_and_real_pnp.py` script. It is very similar to `mover.py` used in the previous tutorial with some minor differences.

- Instead of a ROS Service this node is a ROS Subscriber that uses a ROS ActionClient to send goals.

- Two functions have been added to handle the creation of the RobotMoveActionGoal messages.

	- To send a trajectory command:

	```python
	def send_trajectory_goal(client, trajectory):
	
	    # Build the goal
	    goal = RobotMoveGoal()
	    goal.cmd.Trajectory = trajectory
	    goal.cmd.cmd_type = TRAJECTORY_COMMAND_ID
	
	    client.send_goal(goal)
	    client.wait_for_result()
	
	    return
	```

	- To send a gripper command:
	
	```python
	def send_tool_goal(client, gripper_command):
	    tool_command = ToolCommand()
	    tool_command.tool_id = TOOL_ID
	    tool_command.cmd_type = gripper_command
	    tool_command.gripper_open_speed = GRIPPER_SPEED
	    tool_command.gripper_close_speed = GRIPPER_SPEED
	
	    goal = RobotMoveGoal()
	    goal.cmd.tool_cmd = tool_command
	    goal.cmd.cmd_type = TOOL_COMMAND_ID
	
	    client.send_goal(goal)
	    client.wait_for_result()
	
	    return
	```

- The `pick_and_place` function has been updated to call the two previous functions instead of appening the trajectory or gripper command to a list.

	- Example from `mover.py`
	
	```python
	    previous_ending_joint_angles = grasp_pose.joint_trajectory.points[-1].positions
	    response.trajectories.append(grasp_pose)
	```
	
	- Updated code in `sim_real_pnp.py`
	
	```python
	    previous_ending_joint_angles = grasp_pose.trajectory.joint_trajectory.points[-1].positions
	    send_trajectory_goal(client, grasp_pose)
	```

# The Unity Side
---
We are going to use a new script that mirrors a lot of the functionality of the `TrajectoryPlanner.cs` script with minor differences. Instead of calling a service and waiting for the response  to execute a robot command, a subscription is going to be made to the `niryo_one/commander/robot_action/goal` topic and every command published will be executed.

- Create the subscriber to read the robot command messages and execute them.

```csharp
void Start()
{
    ros.Subscribe<RobotMoveActionGoal>(rosRobotCommandsTopicName, ExecuteRobotCommand);
}
```

- The subscriber callback mimics the robot commander code in the Niryo One ROS stack and determines whether a trajectory or tool command was issued.

```csharp
      void ExecuteRobotCommands(RobotMoveActionGoal robotAction)
    {
        if (robotAction.goal.cmd.cmd_type == TRAJECTORY_COMMAND_EXECUTION)
        {
            StartCoroutine(ExecuteTrajectories(robotAction.goal.cmd.Trajectory.trajectory));
        }
        else if (robotAction.goal.cmd.cmd_type == TOOL_COMMAND_EXECUTION)
        {
            if (robotAction.goal.cmd.tool_cmd.cmd_type == OPEN_GRIPPER)
            {
                Debug.Log("Open Tool Command");
                OpenGripper();
            }
            else if (robotAction.goal.cmd.tool_cmd.cmd_type == CLOSE_GRIPPER)
            {
                Debug.Log("Close Tool Command");
                CloseGripper();
            }
        }
    }
```

- The ExecuteTrajectories function has been updated to accept a single RobotTrajectory object and execute the robot poses one at a time:

```csharp
    private IEnumerator ExecuteTrajectories(RobotTrajectory trajectories)
    {
        // For every robot pose in trajectory plan
        foreach (var point in trajectories.joint_trajectory.points)
        {
            var jointPositions = point.positions;
            float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
            
            // Set the joint values for every joint
            for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            {
                var joint1XDrive  = jointArticulationBodies[joint].xDrive;
                joint1XDrive.target = result[joint];
                jointArticulationBodies[joint].xDrive = joint1XDrive;
            }
            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
        }
    }
```


- The `PublishJoints()` function's only change is that the message is published to a topic instead of being sent as a ServiceMessage.

`ros.Send(rosJointPublishTopicName, request);`


