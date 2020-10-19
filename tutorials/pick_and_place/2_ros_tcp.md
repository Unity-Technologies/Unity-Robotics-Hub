# Pick and Place Tutorial [DRAFT]

This step assumes you have access to a functional ROS workspace. If you do not yet have a working ROS setup, refer to the [Resources](#resources) section below to get started.

Steps covered in this tutorial include creating a TCP connection between Unity and ROS, generating C# scripts from a ROS .msg file, and publishing a Unity GameObject's pose to a ROS topic. 


## Table of Contents
- [Pick and Place Tutorial [DRAFT]](#pick-and-place-tutorial-draft)
  - [Table of Contents](#table-of-contents)
  - [Step 2: Unity & ROS Integration](#step-2-unity--ros-integration)
  - [The Unity Side](#the-unity-side)
  - [The ROS side](#the-ros-side)
  - [Troubleshooting](#troubleshooting)
  - [Resources](#resources)

---

## Step 2: Unity & ROS Integration

- ROS INTRO [PLACEHOLDER]

![](img/2_ros_unity.png)

- Download the provided ROS-side assets from [here](https://drive.google.com/file/d/1IF29DtmP-eX-0iP5gG4aWUs6yNM1iL5p/view?usp=sharing) and unzip the directory. Place the contents inside the source directory of your ROS workspace, e.g. `~/catkin_ws/src`. This package includes skeleton scripts to be filled out in this tutorial, utility classes, MoveIt configs, and the ROS msg and srv definition files.
  
## The Unity Side

- If the PickAndPlace Unity project is not already open, select and open it from the Unity Hub.
  
- The `PickAndPlace.unitypackage` includes a Plugins folder. This contains the MessageGeneration scripts, which have created a new menu option, “RosMessageGeneration.” Select `RosMessageGeneration -> Auto Generate Messages` and select `All Messages in Directory`.

![](img/2_gen.png)
   
- In the Message Auto Generation window that appears, next to the Input Package Path, click `Browse Package…` and navigate to the ros_unity_control directory, e.g. `~/catkin_ws/src/ros_unity_control/`. Select the `msg` folder, and then click `GENERATE!` If this is successful, 4 new C# scripts should populate the `Assets/RosMessages/RosUnityControl/msg` directory: Pose, Rotation, Trajectory, and URJointConfig. Only Pose will be used in this tutorial.
  
    > [PLACEHOLDER] explain what's happening in message generation?

- In the Project window, right click the Assets folder. Create a new folder called Scripts. Then, right click and create a new C# script in `Assets/Scripts` called TCPValues. Double click the script to open. Replace the script with the following:

``` csharp
using System.Collections;
using System.Collections.Generic;
using UnityEngine;
 
public class TCPValues : MonoBehaviour
{
   public string hostName = "192.168.0.66";
   public int hostPort = 10000;
}
``` 

This script serves as a central location for the reference values in the TCP connection that will be made. Other components will look to this one when trying to make the connection. 

- Add the newly created TCPValues component to the `ur3_with_gripper` GameObject by selecting the TCPValues script in the Project window and dragging it onto the `ur3_with_gripper` object in the Hierarchy window.

- The `hostName` should be the IP address of your ROS machine (not the one running Unity).

  - Find the IP address of your ROS machine. On Ubuntu, open a terminal window, and enter `hostname -I`.

  - In the TCP Values component in the Inspector, replace the Host Name value with the IP address of your ROS machine. Ensure that the Host Port is set to `10000`.

- Create a new C# script and name it RosPublisher. Open the file for editing, and replace it with the following:

``` csharp
using RosMessageTypes.RosUnityControl;
using RosMessageTypes.Geometry;
using UnityEngine;
using Pose = RosMessageTypes.Geometry.Pose;
using Quaternion = RosMessageTypes.Geometry.Quaternion;
 
public class RosPublisher : MonoBehaviour
{
  public GameObject cube;
  public float publishMessageFreq = 2.0f;
 
  private TcpConnector tcpCon;
  private float timeElapsed;
  void Start()
  {
     var tcpValues = GetComponent<TCPValues>();
     var hostName = tcpValues.hostName;
     var hostPort = tcpValues.hostPort;
 
     tcpCon = new TcpConnector(hostName, hostPort);
  }
 
  void Update(){
 
     timeElapsed += Time.deltaTime;
      
     if (timeElapsed > publishMessageFreq)
     {
        var pos = cube.transform.position;
        var rot = cube.transform.rotation;
        Pose pose = new Pose(
           new Point(pos.x, pos.y, pos.z),
           new Quaternion(rot.x, rot.y, rot.z, rot.w)
        );
 
        tcpCon.SendMessage("ur3_topic", pose);
 
        timeElapsed = 0;
     }
  }
}
```

This script will create a TCP connection and send messages to the ROS server endpoint--which has yet to be created!

---

## The ROS side

> Note: This project was built using the ROS Melodic distro, and Python 2.

- In your ROS workspace, find the directory `~/catkin_ws/ros_unity_control/scripts`. Select and open `server_endpoint.py` in a text editor. The script is mostly empty--replace the file with the following contents:

``` python
#!/usr/bin/env python
 
import rospy
 
from tcp_endpoint.RosTCPServer import TCPServer
from tcp_endpoint.RosPublisher import RosPublisher
from tcp_endpoint.RosSubscriber import RosSubscriber
from tcp_endpoint.RosService import RosService
 
from ros_unity_control.msg import *
from ros_unity_control.srv import *
 
 
def main():
   ros_tcp_ip = '192.168.0.66'
   ros_tcp_port = 10000
   ros_node_name = 'TCPServer'
   buffer_size = 1024
   connections = 10
   unity_machine_ip = '192.168.0.48'
   unity_machine_port = 5005
 
   rospy.init_node(ros_node_name, anonymous=True)
   rate = rospy.Rate(10)  # 10hz
 
   source_destination_dict = {
       'ur3_topic': RosPublisher('ur3_topic', Pose, queue_size=10)
   }
 
   tcp_server = TCPServer(ros_tcp_ip, ros_tcp_port, ros_node_name, source_destination_dict, buffer_size, connections)
   tcp_server.start()
   rospy.spin()
 
 
if __name__ == "__main__":
   main()
```

This script imports the necessary dependencies from the tcp_endpoint package (for creating the TCP connection with Unity), defines the address and port values, and starts the server. `rospy.spin()` ensures the node does not exit until it is shut down.

  - Replace the `ros_tcp_ip` value with the value of your ROS IP address--this should match the one set in TCPValues in Unity. Then, find the IP address of your local machine (the one running Unity). On a Mac, open `System Preferences > Network`. The IP address should be listed on the active connection. Replace the `unity_machine_ip` value with your local machine’s IP address. 
  
- Open a terminal in the ROS workspace. Run `roscore`.
  
- Open a second terminal in the ROS workspace. `rosrun` the new server_endpoint, e.g. `rosrun ros_unity_control server_endpoint.py`.

- Open a third terminal in the ROS workspace. Run `rostopic echo ur3_topic`.

- Return to Unity, and press Play. The `rostopic echo ur3_topic` console should print the pose of the cube every 2.0 seconds. ROS and Unity have now successfully connected!

![](img/2_echo.png)

---

## Troubleshooting

- If the error `[rosrun] Found the following, but they're either not files, or not executable: server_endpoint.py` appears, the Python script may need to be marked as executable via `chmod +x ~/catkin_ws/src/ros_unity_control/scripts/server_endpoint.py`.
  
- If Unity fails to find a network connection, ensure that the ROS IP address is entered into the Host Name in the Motion Planning Service component in Unity. Additionally, ensure that the `ros_tcp_ip` address matches in the `server_endpoint.py` value, and that `unity_machine_ip` is your Unity machine's IP.

---

## Resources

- Setting up a ROS workspace:
   -  http://wiki.ros.org/ROS/Installation
   > Note: this tutorial was made using ROS Melodic.
   -  http://wiki.ros.org/ROS/Tutorials/InstallingandConfiguringROSEnvironment
   - http://wiki.ros.org/catkin/Tutorials/create_a_workspace

---

Proceed to [Step 3](3_naive.md).