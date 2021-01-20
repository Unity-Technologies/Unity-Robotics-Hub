# Pick-and-Place Tutorial: Part 4

This part is going to be a little different than the previous tutorials in that it will utilize a real Niryo One robot. Although we assume you have completed the previous three parts ([Part 1](1_urdf.md), [Part 2](2_ros_tcp.md), [Part 3](3_pick_and_place.md)), we do not assume that you have access to a Niryo One outside of simulation. As such this tutorial should mostly be used as a reference for how to simulate the behaviour of a robot operating in real life.

**Table of Contents**

  - [Niryo One Information](#niryo-one-information)
  - [RobotMoveGoal Parameters](#robotmovegoal-parameters)
  - [Differences From Part 3](#differences-fromt-part-3)
  - [The ROS Side](#the-ros-side)
  - [The Unity Side](#the-unity-side)
  	- [Key Differences](#key-differences)
  	- [Setting Up Unity Scene](#settinig-up-unity-scene)
  - [Setting Up Niryo One](#setting-up-niryo-one)
  	- [Update Niryo One URDFs](#update-niryo-one-urdfs)
  	- [Add niryo_moveit Package](#add-niryo_moveit-package)
  - [Execution](#execution)

  
# Niryo One Information

The best source of information for the Niryo One is the [User Manual](https://niryo.com/docs/niryo-one/user-manual/complete-user-manual/). It contains a lot of general information about the Niryo One like instructions on how to connecct to the Niryo One, including passwords, and how to use the Niryo One Studio desktop application.

For this tutorial we will be using the full [Niryo One ROS](https://niryo.com/docs/niryo-one/developer-tutorials/get-started-with-the-niryo-one-ros-stack/) stack running on the Niryo One robot. 

From the Niryo One ROS documentation:
> The commander package exposes an action server named “niryo_one/commander/robot_action”, with a “RobotMoveCommand” message. This is probably what you’re looking for if you want to send direct command to the robot using ROS.

Using the `RobotMoveCommand` action server we can use an [action client](http://wiki.ros.org/actionlib_tutorials/Tutorials/Writing%20a%20Simple%20Action%20Client%20%28Python%29) to send `RobotMoveGoal`s to the Niryo One for execution.

> If you are unfamiliar with ROS actions you can read more about them [here](http://wiki.ros.org/actionlib).

> Commands can also be executed on the Niryo One by publishing `RobotMoveActionGoal` messages directly to the `niryo_one/commander/robot_action/goal` topic. However, the default behavior of the action server allows only one action to be executed at a time, so any new actions received by the action server will replace any currently executing actions.

## RobotMoveGoal Parameters

So far we know that the Niryo One robot action server is expecting `RobotMoveGoal` messages. Looking at the `RobotMoveGoal` message type we see that it contains a single parameter, `RobotMoveCommand`. Furthermore, looking at the `RobotMoveCommand` message we see that it is used as a catch all for executing a variety of commands on the robot.

### RobotMoveGoal.RobotMoveCommand

Since we are only interested in executing trajectory and gripper commands we will only focus on the following parameters:

- `int32 cmd_type`
	- Which type of robot command to execute, trajectory or tool command
- `niryo_one_msgs/TrajectoryPlan Trajectory`
	- The trajectory to be executed
- `niryo_one_msgs/ToolCommand tool_cmd`
	- The end effector command to be executed

Since the `cmd_type` variable is used to determine which command in the `RobotMoveCommand` message is to be executed, we will need to find the appropriate values to exectue trajectory or tool commands.

We can find this information in the `niryo_one_ros/niryo_one_commander/src/niryo_one_commander/command_type.py` file:

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

We can see that the `cmd_type` variable in `RobotMoveCommand` will need to be `7` to execute a trajectory and `6` to execute a tool command.

### RobotMoveGoal.RobotMoveCommand.ToolCommand

To execute an end effector command on the Niryo One, it requires a `ToolCommand` object. Much like `RobotMoveCommand`, the `ToolCommand` message type is a catch all for gripper commands as the Niryo One supports a variety of [grippers](https://niryo.com/niryo-one-accessories/).

We are only interested in opening and closing the standard gripper so we will only use the following parameters:

- `uint8 tool_id`
	- Which end effector is being used
- `uint8 cmd_type`
	- Which command to execute. Ex. open or close
- `uint16 gripper_close_speed`
	- How fast should the gripper close
- `uint16 gripper_open_speed`
	- How fast should the gripper open

Just like the `cmd_type` parameter in `RobotMoveCommand` we will need to look at `niryo_one_ros/niryo_one_tools/config/end_effectors.yaml` to find the appropriate values to use.

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

From here we see that the `RobotMoveGoal.RobotMoveCommand.ToolCommand.cmd_type` variable will need to be `2` to close the gripper and `1` to open it. 


## Differences From Part 3
**Part 3 Message Flow:**

**TODO: Diagrams**

- `TrajectoryPlanner` sends ServiceMessage with robot pose, target cube coordinates, and target destination coordinates to `mover.py`
- `mover.py` calculates the four trajectories, adds them to an array, and returns the array to `TrajectoryPlanner`
- `TrajectoryPlanner` then executes the trajectories one at a time

**The New Flow:**

- `RealSimPickAndPlace.cs` publishes robot pose, target cube coordinates, and target destination coordinates to `sim_real_pnp` topic
- `sim_and_real_pnp.py,`as a ROS subscriber node, reads from the `sim_real_pnp` topic, plans the trajectories, and sends the action goals one at a time.

- **Simultaneously**
	- Simulated Niryo One, `RealSimPickAndPlace.cs`, subscriber reads from the action goal topic, `robot_action/goal`, and executes the trajectory
	- Real Niryo One reads from the action goal topic, `robot_action/goal`, and executes trajectory


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

- The `pick_and_place` function has been updated to call the two previous functions instead of appending the trajectory or gripper command to a list.

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
Using the same scene from [Part 3](3_pick_and_place.md), we are going to use a new script, `RealSimPickAndPlace.cs`, that mirrors a lot of the functionality of the `TrajectoryPlanner.cs` script. 


## Key Differences
Instead of calling a service and waiting for the response  to execute a robot command, a subscription is going to be made to the `niryo_one/commander/robot_action/goal` topic and every command published will be executed upon reception.

- A subscriber is create on Start to read the robot command messages and execute them.

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


## Setting Up Unity Scene
1. In Unity, Select Robotics -> ROS Settings from the top menu bar and update the ROS IP Address to that of the Niryo One.

1. Select the Publisher GameObject and add the `RealSimPickAndPlace` script as a component.

1. Note that the RealSimPickAndPlace component shows its member variables in the Inspector window, which need to be assigned. 

    Once again, drag and drop the `Target` and `TargetPlacement` objects onto the Target and Target Placement Inspector fields, respectively. Assign the `niryo_one` robot to the Niryo One field. 

![](img/4_script.png)

1. Select the previously made Button object in Canvas/Button, and scroll to see the Button component. Under the `OnClick()` header, click the dropdown where it is currently assigned to the SourceDestinationPublisher.Publish(). Replace this call with RealSimPickAndPlace > `PublishJoints()`.

![](img/4_button.png)

1. The Unity side is now ready to communicate with ROS on the Niryo One!


# Setting Up Niryo One
---

## Update Niryo One URDFs
- To account for the simulated Niryo One's position on a table, the URDF files on the Niryo One will need to be updated to reflect this change.

	- The two files that will need to be updated are `niryo_one.urdf.xacro`  and `without_mesh_niryo_one.urdf.xacro` located in the `/home/niryo/catkin_ws/src/niryo_one_description/urdf/v2` directory.
	- Look for the joint named `joint_world` and update the `origin`'s `xyz` to `0 0 0.63` to reflect that the simulated Niryo is placed at `0.63` on the Z axis.
	
	```xml
	    <joint name="joint_world" type="fixed">
	        <parent link="world" />
	        <child link="base_link" />
	        <origin xyz="0 0 0.63" rpy="0 0 0" />
	    </joint>
	```

## Add niryo_moveit Package
- Update the `niryo_moveit/config/params.yml` file's `ROS_IP` parameter to match that of the Niryo One.

> An easy way to find the Niryo One's IP address is to connect to it using the Niryo One Studio application

- Copy the `niryo_moveit` package to the `catkin_ws` directory on the Niryo One's catkin workspace at `/home/niryo/catkin_ws` and run the `catkin_make` command.

> Using the SCP command to transfer the `niryo_moveit` package might look something like, `scp -r ~/PATH/TO/niryo_moveit niryo@NIRYO_IP_ADDRESS:/home/niryo/catkin_ws/src`

# Execution
1. Use the `part_4.launch` file to setup the ROS params and start the `server_endpoint` and `sim_real_pnp` scripts.
	- `roslaucnh niryo_moveit part_4.launch`

1. Return to the Unity Editor and press Play. Press the UI Button to send the joint configurations to ROS on the Niryo One, and watch the robot arm move simultaneously in simulation and real life!



