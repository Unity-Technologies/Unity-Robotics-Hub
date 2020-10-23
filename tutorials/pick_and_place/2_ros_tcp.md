# Pick and Place Tutorial [DRAFT]

This step assumes you have access to a functional ROS workspace. If you do not yet have a working ROS setup, refer to the [Resources](#resources) section below to get started.

Steps covered in this tutorial include creating a TCP connection between Unity and ROS, generating C# scripts from a ROS msg and srv files, and setting Unity joint values via a ROS Service.


- [Pick and Place Tutorial [DRAFT]](#pick-and-place-tutorial-draft)
  - [Step 2: Unity & ROS Integration](#step-2-unity--ros-integration)
  - [The Unity Side](#the-unity-side)
  - [The ROS side](#the-ros-side)
    - [Server Endpoint](#server-endpoint)
    - [Mover](#mover)
    - [MoveIt [PLACEHOLDER]](#moveit-placeholder)
  - [Troubleshooting](#troubleshooting)
  - [Resources](#resources)

---

## Step 2: Unity & ROS Integration

- ROS INTRO [PLACEHOLDER]

![](img/2_ros_unity.png)

- Download the provided ROS-side assets from [PLACEHOLDER]() and unzip the directory. Place the contents inside the source directory of your ROS workspace, e.g. `~/catkin_ws/src`. This package includes Python scripts, MoveIt configs, and the ROS msg and srv definition files.
  
## The Unity Side

- If the current Unity project is not already open, select and open it from the Unity Hub.
  
- The `PLACEHOLDER.unitypackage` includes a Plugins folder. This contains the MessageGeneration scripts, which have created a new menu option, “RosMessageGeneration.” Select `RosMessageGeneration -> Auto Generate Messages` and select `All Messages in Directory`.

![](img/2_gen.png)
   
- In the Message Auto Generation window that appears, next to the Input Package Path, click `Browse Package…` and navigate to the ros_unity_control directory, e.g. `~/catkin_ws/src/niryo_moveit/`. Select the `msg` folder, and then click `GENERATE!` If this is successful, 1 new C# script should populate the `Assets/RosMessages/NiryoMoveit/msg` directory: NiryoMoveitJoints.
  
    > [PLACEHOLDER] explain what's happening in message generation?

- In the Project window, right click the Assets folder. Create a new folder called Scripts. Then, right click and create a new C# script in `Assets/Scripts` called RosConnect. Double click the script to open. Replace the script with the following:

``` csharp
using System.Collections;
using System.Linq;
using System.Net.Sockets;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;

public class RosConnect : MonoBehaviour
{
   // ROS Connector
   private TcpConnector tcpCon;

   private readonly float jointAssignmentWait = 0.06f;
   private readonly float poseAssignmentWait = 1f;

   // Variables required for ROS communication
   public string rosServiceName = "niryo_moveit";
   public string hostName = "192.168.50.149";
   public int hostPort = 10000;
   public int connectionTimeout = 10;

   // GameObjects used to get Articulation Bodies
   public GameObject[] jointGameObjects;

   // Articulation Bodies
   private ArticulationBody[] jointArticulationBodies;
   
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
   }

   // PLACEHOLDER
}
``` 

This script will communicate with ROS, and will soon be able to receive a set of joint values to set the robot arm to.

- Add the newly created RosConnect component to the `niryo_one` GameObject by selecting the RosConnect script in the Project window and dragging it onto the `niryo_one` object in the Hierarchy window.

- The `hostName` should be the IP address of your ROS machine (not the one running Unity).

  - Find the IP address of your ROS machine. In Ubuntu, open a terminal window, and enter `hostname -I`.

  - In the RosConnect component in the Inspector, replace the Host Name value with the IP address of your ROS machine. Ensure that the Host Port is set to `10000`.

---

## The ROS side

> Note: This project was built using the ROS Melodic distro, and Python 2.

- The provided files require the following packages to be installed:

   ```bash
   sudo apt-get install ros-melodic-robot-state-publisher ros-melodic-moveit ros-melodic-rosbridge-suite ros-melodic-joy ros-melodic-ros-control ros-melodic-ros-controllers ros-melodic-tf2-web-republisher
   ```

   ```bash
   sudo -H pip install jsonpickle
   ```

### Server Endpoint

- In your ROS workspace, find the directory `~/catkin_ws/niryo_moveit/scripts`. Note the file `server_endpoint.py`. This script imports the necessary dependencies from the tcp_endpoint package (for creating the TCP connection with Unity), defines the address and port values, and starts the server. `rospy.spin()` ensures the node does not exit until it is shut down.

  <!-- - Replace the `ros_tcp_ip` value with the value of your ROS IP address--this should match the one set in TCPValues in Unity. Then, find the IP address of your local machine (the one running Unity). On a Mac, open `System Preferences > Network`. The IP address should be listed on the active connection. Replace the `unity_machine_ip` value with your local machine’s IP address.  -->
  
- If you have not already built and sourced the catkin workspace since importing the new ROS packages, run `cd ~/catkin_ws/ && catkin_make && source devel/setup.bash`. Ensure there are no errors.

- Open a new terminal in the ROS workspace and navigate to your catkin workspace. Run:
   ```bash
   roscore &
   ```

Once ROS Core has started, it will print `started core service [/rosout]` to the terminal window.

- Note that in the `server_endpoint`, the script fetches parameters for the TCP connection. You will need to know the IP address of your ROS machine as well as the IP address of the machine running Unity. 
   - The ROS machine IP, i.e. `ROS_IP` should be the same value as the one set as `hostName` on the RosConnect component in Unity.
   - Finding the IP address of your local machine (the one running Unity), i.e. `UNITY_IP` depends on your operating system. 
     - On a Mac, open `System Preferences > Network`. The IP address should be listed on the active connection.
     - On Windows, open `Wi-Fi network` on the taskbar, and open `Properties`. The IP address should be listed next to "IPv4 address."

- Set the parameter values by running the following commands:

   ```bash
   rosparam set ROS_IP <your ROS IP>
   rosparam set ROS_TCP_PORT 10000
   rosparam set UNITY_IP <your Unity IP>
   rosparam set UNITY_SERVER_PORT 5005
   ```
   e.g.,

   ```bash
   rosparam set ROS_IP 192.168.50.149
   rosparam set ROS_TCP_PORT 10000
   rosparam set UNITY_IP 192.168.50.13
   rosparam set UNITY_SERVER_PORT 5005
   ```

- Once the parameter values have been set, you can run the server_endpoint. In this same terminal, run:
  
   ```bash
   rosrun niryo_moveit server_endpoint.py
   ```
Once the server_endpoint has started, it will print something similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000`.

### Mover

- Note the file `mover.py`. PLACEHOLDER DESCRIPTION

- Open a second terminal in the ROS workspace. `rosrun` this mover script, e.g. 
   ```bash
   rosrun niryo_moveit mover.py
   ```

Once this process is ready, it will print `Ready to motion plan!` to the console.


### MoveIt [PLACEHOLDER]

- Open a third terminal in the ROS workspace to start the Niryo Moveit Node. This is the node that will actually run the motion planning and output a trajectory. Run:
   ```bash
   roslaunch niryo_moveit demo.launch
   ```
This may print out various error messages regarding the controller_spawner, such as `[controller_spawner-4] process has died`. These messages are safe to ignore, so long as the final message to the console is `You can start planning now!`.


- Return to Unity, and press Play. PLACEHOLDER for what happens here
  
ROS and Unity have now successfully connected!

![](img/2_echo.png)

---

## Troubleshooting

- If the error `[rosrun] Found the following, but they're either not files, or not executable: server_endpoint.py` appears, the Python script may need to be marked as executable via `chmod +x ~/catkin_ws/src/niryo_moveit/scripts/server_endpoint.py`.
  
- If Unity fails to find a network connection, ensure that the ROS IP address is entered correctly as the Host Name in the RosConnector component in Unity. 

---

## Resources

- Setting up a ROS workspace:
   -  http://wiki.ros.org/ROS/Installation
   > Note: this tutorial was made using ROS Melodic.
   -  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
   - http://wiki.ros.org/catkin/Tutorials/create_a_workspace

---

Proceed to [Step 3](3_naive.md).