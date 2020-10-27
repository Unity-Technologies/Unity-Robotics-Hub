# Pick and Place Tutorial [DRAFT]

This step assumes you have access to a functional ROS workspace and that the previous two steps ([Step 1](1_urdf.md), [Step 2](2_ros_tcp.md)) have been completed.

Steps covered in this tutorial includes invoking a motion planning service in ROS, moving a Unity Articulation Body based on the calculated trajectory, and controlling a gripping tool to successfully grasp a cube.

## Table of Contents
- [Pick and Place Tutorial [DRAFT]](#pick-and-place-tutorial-draft)
  - [Table of Contents](#table-of-contents)
  - [Step 3: Naive Pick & Place](#step-3-naive-pick--place)
  - [The Unity Side](#the-unity-side)
  - [The ROS Side](#the-ros-side)
  - [Unity & ROS Communication](#unity--ros-communication)
  - [Resources](#resources)
  - [Troubleshooting](#troubleshooting)
    - [Errors and Warnings](#errors-and-warnings)
    - [Hangs, Timeouts, and Freezes](#hangs-timeouts-and-freezes)
    - [Miscellaneous Issues](#miscellaneous-issues)

---

## Step 3: Naive Pick & Place

## The Unity Side

- If you have not already cloned this [PLACEHOLDER] repository, do so now, and follow the steps in [Step 1](1_urdf.md) to set up the Unity project, and [Step 2](2_ros_tcp.md) to integrate ROS with Unity. 

<!-- - Note the SourceDestinationPublisher script. This script will communicate with ROS, grabbing the positions of the target and destination objects and sending it to the ROS Topic `"SourceDestination_input"`. On `Start()`, the TCP connector is instantiated with a ROS host name and port. The `Publish()` function is defined as follows: -->

- If the current Unity project is not already open, select and open it from the Unity Hub.

- Note the Assets/Scripts/RosConnect.cs script. PLACEHOLDER description

```csharp
private void CloseGripper()
{
    var leftDrive = leftGripper.xDrive;
    var rightDrive = rightGripper.xDrive;

    leftDrive.target = -0.01f;
    rightDrive.target = 0.01f;

    leftGripper.xDrive = leftDrive;
    rightGripper.xDrive = rightDrive;
}

private void OpenGripper()
{
    var leftDrive = leftGripper.xDrive;
    var rightDrive = rightGripper.xDrive;

    leftDrive.target = 0;
    rightDrive.target = 0;

    leftGripper.xDrive = leftDrive;
    rightGripper.xDrive = rightDrive;
}
```

PLACEHOLDER 

```csharp
private IEnumerator PrintTrajectories(MoverServiceResponse response)
{
    if (response.trajectories != null)
    {
        for (int poseIndex  = 0 ; poseIndex < response.trajectories.Length; poseIndex++)
        {
            for (int jointConfigIndex  = 0 ; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
            {
                var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
                
                for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                {
                    var joint1XDrive  = jointArticulationBodies[joint].xDrive;
                    joint1XDrive.target = result[joint];
                    jointArticulationBodies[joint].xDrive = joint1XDrive;
                }
                yield return new WaitForSeconds(jointAssignmentWait);
            }

            if (poseIndex == (int)Poses.Grasp)
                CloseGripper();
            
            yield return new WaitForSeconds(poseAssignmentWait);
        }
        // Open Gripper at end of sequence
        OpenGripper();
    }
}
```

```csharp
void Start()
{ 
    TcpClient client = new TcpClient();

    // Instantiate the connector with ROS host name and port.
    tcpCon = new TcpConnector(hostName, hostPort, serviceResponseRetry: 10, serviceResponseSleep: 1000);
    
    jointArticulationBodies = new ArticulationBody[jointGameObjects.Length];
    // Setup articulation bodies
    for (int i = 0; i < jointGameObjects.Length; i++)
    {
        jointArticulationBodies[i] = jointGameObjects[i].GetComponent<ArticulationBody>();
    }

    leftGripper = leftGripperGO.GetComponent<ArticulationBody>();
    rightGripper = rightGripperGO.GetComponent<ArticulationBody>();
}
```

- Select the RosConnector GameObject. Disable the SourceDestinationPublisher component by toggling off the script's checkmark in the Inspector window. Add the RosConnect script to the RosConnector object.

- Note that the RosConnect component shows its member variables in the Inspector window, which are unassigned. Drag and drop the Target and TargetPlacement objects onto the Target and Target Placement Inspector fields, respectively.

- In the Search bar in the Hierarchy, search for "_gripper". Drag the left_gripper object to the PLACEHOLDER Right Gripper GO field, and the right_gripper object to the Left Gripper GO field.

- Expand the Joint Game Objects list. In this order, drag and drop the following objects into the Joint Game Objects list: shoulder_link, arm_link, elbow_link, forearm_link, wrist_link, hand_link. 
  - This can be done by expanding the niryo_one Hierarchy through `niryo_one/world/base_link/shoulder_link/arm_link/...`, or by searching for these objects in the Hierarchy.

![](img/2_joints.gif)

- PLACEHOLDER reassign button OnClick

- The Unity side is now ready to communicate with ROS to motion plan!

---

## The ROS Side

> Note: This project was built using the ROS Melodic distro, and Python 2.

- The provided files require the following packages to be installed; run the following if the packages are not already present:

   ```bash
   sudo apt-get install ros-melodic-robot-state-publisher ros-melodic-moveit ros-melodic-rosbridge-suite ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-tf2-web-republisher
   ```

   ```bash
   sudo -H pip install jsonpickle
   ```

- PLACEHOLDER describe mover.py

- discussion on moveit configs 

- If you have not already built and sourced the catkin workspace since importing the new ROS packages, run `cd ~/catkin_ws/ && catkin_make && source devel/setup.bash`. Ensure there are no errors.

--- 

## Unity & ROS Communication

- The ROS side is now ready to interface with the Unity side! Open a new terminal window and navigate to your catkin workspace. Start ROS Core, set the parameter values, and begin the server_endpoint as follows:

``` bash
cd ~/catkin_ws/ && source devel/setup.bash
roscore &
rosparam set ROS_IP <your ROS IP>
rosparam set ROS_TCP_PORT 10000
rosparam set UNITY_IP <your Unity IP>
rosparam set UNITY_SERVER_PORT 5005
rosrun niryo_moveit server_endpoint.py
```

Once ROS Core has started, it will print `started core service [/rosout]` to the terminal window. Once the server_endpoint has started, it will print something similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000`.

- Open a new terminal window and start the Motion Planning Node. This is the node that controls which pose the UR3 will travel to within the Unity Editor.

``` bash
cd ~/catkin_ws/ && source devel/setup.bash

# Start motion planning script
rosrun niryo_moveit mover.py
```

Once this process is ready, it will print `Ready to motion plan!` to the console.

- Open a new terminal window and start the Moveit Node. This is the node that will actually run motion planning computations and load the configuration.

``` bash
cd ~/catkin_ws/ && source devel/setup.bash

# Start the planning environment
roslaunch niryo_moveit demo.launch
```

This may print out various error messages regarding the controller_spawner, such as `[controller_spawner-4] process has died`. These messages are safe to ignore, so long as the final message to the console is `You can start planning now!`.

- The ROS side of the setup is ready! 

- Return to the Unity Editor and press Play. Press the UI Button to send the joint configurations to ROS, and watch the robot arm pick and place the cube! 
  - The target object and placement positions can be moved around during runtime for different trajectory calculations. 
  
---

## Resources

- [MoveIt!](https://github.com/ros-planning/moveit)
- All of the launch and config files used were copied from [Niryo One ROS Stack](https://github.com/NiryoRobotics/niryo_one_ros) and edited to suit our reduced use case

---

## Troubleshooting

### Errors and Warnings

- If the motion planning script throws a `RuntimeError: Unable to connect to move_group action server 'move_group' within allotted time (5s)`, ensure the `roslaunch ur3_with_gripper_moveit gazebo.launch sim:=true` process launched correctly and has printed `You can start planning now!`.
  
- `[ WARN] [1600887082.269260191, 1270.407000000]: Fail: ABORTED: No motion plan found. No execution attempted. WARNING: the motion planner failed because unknown error handler name 'rosmsg'` This is due to a bug in an outdated version. Try running `sudo apt-get update && sudo apt-get upgrade` to upgrade.

### Hangs, Timeouts, and Freezes

- If Unity fails to find a network connection, ensure that the ROS IP address is entered into the Host Name in the Motion Planning Service component in Unity. Additionally, ensure that the `ros_tcp_ip` address matches in the `server_endpoint.py` value, and that `unity_machine_ip` is your Unity machine's IP.
  
- The VM-side calculations may take longer than expected on different machines. If the Unity console throws an error saying `No data available on network stream after <number> attempts`, open the `Unity3D/Assets/Scripts/ROS Services/PoseEstimationService.cs` script. Edit the number after `serviceResponseRetry`.
    > E.g. `tcpCon = new TcpConnector(hostName, hostPort, networkTimeout: 3000, serviceResponseRetry: 30, serviceResponseSleep: 2000);`

- If running `roslaunch ur3_with_gripper_moveit gazebo.launch sim:=true` does not appear to throw breaking errors but does not proceed to print `You can start planning now!`, wait for up to five minutes. If it still does not proceed, the memory allowance may need to be increased.

### Miscellaneous Issues

- If the robot appears loose/wiggly or is not moving with no console errors, ensure that the Stiffness and Damping values on the Controller script of the `ur3_with_gripper` object are set to `10000` and `1000`, respectively.

- Before entering Play mode in the Unity Editor, ensure that all ROS processes are still running. The `server_endpoint.py` script may time out, and will need to be re-run.