# Pick-and-Place Tutorial: Part 4

This part assumes you have access to a functional ROS workspace and that the previous three parts ([Part 1](1_urdf.md), [Part 2](2_ros_tcp.md), [Part 3](2_pick_and_place.md)) have been completed.

**NOTE:**
This tutorial uses the full [Niryo One ROS](https://niryo.com/docs/niryo-one/developer-tutorials/get-started-with-the-niryo-one-ros-stack/) stack running on the Niryo One robot.

> Since most people will not have access to a Niryo One robot this tutorial will mostly be written as reference.


The Niryo One ROS stack uses a ROS Action to drive the physical motors of the robot so we are going to send a Goal to an Action Server without creating Action Client.

Part 3 flow:

- `TrajectoryPlanner` sends ServiceMessage with robot pose, target cube coordinates, and target destination coordinates to `mover.py`
- `mover.py` calculates the four trajectories, adds them to an array, and returns the array to `TrajectoryPlanner`
- `TrajectoryPlanner` then executes the trajectories one at a time

The new flow:

- `NiryoSubscriber.cs` publishes robot pose, target cube coordinates, and target destination coordinates to `sim_real_pnp` topic
- `sim_and_real_pnp.py,`as a ROS subscriber node, reads from the `sim_real_pnp` topic and:
	- Plans pre-grasp trajectory and publishes it to `robot_action/goal` topic
	- Plans grasp trajectory and publishes it to `robot_action/goal` topic
	- Plans pick up trajectory and publishes it to `robot_action/goal` topic
	- Plans placement trajectory and publishes it to `robot_action/goal` topic
- Simulated Niryo One, `NiryoSubscriber.cs`, subscriber reads from `robot_action/goal` topic and executes the trajectory
- Real Niryo One reads from `robot_action/goal`  topic and executes trajectory

**Table of Contents**

  - [The ROS Side](#the-ros-side)
  - [The Unity Side](#the-unity-side)


## Part 4: Sim-and-Real

Robot commands can be executed on the Niryo One by publishing `RobotMoveActionGoal` messages to the `niryo_one/commander/robot_action/goal` topic.

`RobotMoveActionGoal` contains only a parameter `RobotMoveCommand` which is a catch all message for executing a variety of commands on the robot. We are only interested in executing trajectories and gripper commands so we will only focus on the follow parameters:

- `int32 cmd_type`
	- Which command to execute
- `niryo_one_msgs/TrajectoryPlan Trajectory`
	- A trajectory to be executed
- `niryo_one_msgs/ToolCommand tool_cmd`
	- The end effector command to be executed

We are only interested in opening and closing the gripper so we will only use thhe following parameters:

- `uint8 tool_id`
	- Which end effector is being used
- `uint8 cmd_type`
	- Which command to execute. Ex. open or close
- `uint16 gripper_close_speed`
	- How fast should the gripper close
- `uint16 gripper_open_speed`
	- How fast should the gripper close


Looking at the `command_type.py` file we find the `cmd_type` variable in `RobotMoveCommand`
 needs to be `7` to execute a trajectories and `6` to execute tool commands.

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


Referenccing `end_effectors.yaml` shows that the `cmd_type` variable in `ToolCommand` needs to be `2` to close the gripper and `1` to open it. 

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


# The ROS Side
---
The following lines in `server_endpoint.py` will be used for this tutorial.


```python
'niryo_one/commander/robot_action/goal': RosSubscriber('niryo_one/commander/robot_action/goal', RobotMoveActionGoal, tcp_server),
```


```python
'sim_real_pnp': RosPublisher('sim_real_pnp', MoverServiceRequest)
```

`sim_and_real_pnp.py` is 


Two functions have been added that handle the creation of the RobotMoveActionGoal messages.


```python
def publish_trajectory(publisher, trajectory):
    action_goal = RobotMoveActionGoal()
    action_goal.goal.cmd.Trajectory = trajectory
    action_goal.goal.cmd.cmd_type = TRAJECTORY_COMMAND_ID
    publisher.publish(action_goal)

```

```python
def publish_tool_command(publisher, gripper_command):
    tool_command = ToolCommand()
    tool_command.tool_id = TOOL_ID
    tool_command.cmd_type = gripper_command
    tool_command.gripper_open_speed = GRIPPER_SPEED
    tool_command.gripper_close_speed = GRIPPER_SPEED

    action_goal = RobotMoveActionGoal()
    action_goal.goal.cmd.tool_cmd = tool_command
    action_goal.goal.cmd.cmd_type = TOOL_COMMAND_ID
    publisher.publish(action_goal)

```



# The Unity Side
---
We are going to use a new script that mirrors a lot of the functionality of the `TrajectoryPlanner.cs` script with minor differences. Instead of calling a service and waiting for the trajectory response a subscriber is going to subscribe to a topic where the trajectories will be published.

Create the subscriber to pull the robot command messages and start a coroutine that will handle the execution of the messages read by the subscriber.

```csharp
void Start()
{
    ros.Subscribe<RobotMoveActionGoal>(rosRobotCommandsTopicName, AppendRobotCommands);
    StartCoroutine(ExecuteRobotCommand());
}
```

The subscriber's callback function's only job is to append robot commands to a list read from the `RobotMoveActionGoal` topic.

```csharp
void AppendRobotCommands(RobotMoveActionGoal robotAction)
{
    moveActionGoals.Add(robotAction);
}
```

The coroutine 

```csharp
    IEnumerator ExecuteRobotCommand()
    {
        for (;;)
        {
            if (moveActionGoals.Count > 0)
            {
                if (moveActionGoals[0].goal.cmd.cmd_type == TRAJECTORY_COMMAND_EXECUTION)
                {
                    StartCoroutine(ExecuteTrajectories(moveActionGoals[0].goal.cmd.Trajectory.trajectory));
                }
                else if (moveActionGoals[0].goal.cmd.cmd_type == TOOL_COMMAND_EXECUTION)
                {
                    if (moveActionGoals[0].goal.cmd.tool_cmd.cmd_type == OPEN_GRIPPER)
                    {
                        OpenGripper();
                    }
                    else if (moveActionGoals[0].goal.cmd.tool_cmd.cmd_type == CLOSE_GRIPPER)
                    {
                        CloseGripper();
                    }
                }
                // Remove executed command
                moveActionGoals.RemoveAt(0);
            }

            yield return new WaitForSeconds(commandExecutionSleep);
        }
    }
```

The ExecuteTrajectories function has been updated to accept a single RobotTrajectory object and execute the :

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


The `PublishJoints()` function's only change is that the message is published to a topic instead of being sent as a ServiceMessage.

`ros.Send(rosJointPublishTopicName, request);`


