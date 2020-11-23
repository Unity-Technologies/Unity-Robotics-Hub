# ROS–Unity Integration: Subscriber

Create a simple Unity scene which subscribes to a [ROS topic](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics#ROS_Topics) to change the colour of a GameObject.

**NOTE:** If following from [Publisher](publisher.md) tutorial proceed to [Setting Up Unity Scene](subscriber.md#setting-up-unity-scene) step.

## Setting Up ROS

(You can skip this if you already did the [ROS–Unity Integration Publisher](publisher.md) tutorial.)

- Copy the `tutorials/ros_packages/robotics_demo` folder of this repo into the `src` folder in your Catkin workspace.

- Follow the [ROS–Unity Initial Setup](setup.md) guide.

- Open a new terminal window, navigate to your Catkin workspace, and run the following commands:
  
   ```bash
    source devel/setup.bash
	rosrun robotics_demo server_endpoint.py
   ```

Once the server_endpoint has started, it will print something similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000`.

## Setting Up Unity Scene
- Generate the C# code for `UnityColor` message by going to `RosMessageGeneration` -> `AutoGenerateMessages` -> `Single Message...`
- Set the input file path to `PATH/TO/Unity-Robotics-Hub/tutorials/ros_packages/robotics_demo/msg/UnityColor.msg` and click `GENERATE!`
    - The generated file will be saved in the default directory `Assets/RosMessages/msg`
- Create a script and name it `RosSubscriberExample.cs`
- Paste the following code into `RosSubscriberExample.cs`
	- **Note** Script can be found at `tutorials/ros_unity_integration/unity_scripts`

```csharp
using UnityEngine;
using RosColor = RosMessageTypes.RoboticsDemo.UnityColor;

public class RosSubscriberExample : MonoBehaviour
{
    public ROSConnection ros;
    public GameObject cube;

    void Start()
    {
        ros.Subscribe<RosColor>("color", ColorChange);
    }

    void ColorChange(RosColor colorMessage)
    {
        cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
    }
}
```

- Create an empty GameObject, name it `RosConnection` and attach the `Plugins/TcpConnector/ROSConnection` script. (Or, if you're reusing the same scene from the Publisher tutorial, you can just keep the existing RosConnection object.)
	- Change the host name and port to match the ROS IP and port variables defined when you set up ROS.
	- The IP for Unity to listen on should be determined automatically, but if you're having trouble, you can set it manually in the `Override Unity IP` field. Finding the IP address of your local machine (the one running Unity) depends on your operating system.
		- On a Mac, open `System Preferences > Network`. Your IP address should be listed on the active connection.
		- On Windows, click the Wi-Fi icon on the taskbar, and open `Properties`. Your IP address should be listed near the bottom, next to "IPv4 address."

- Create an empty GameObject and name it `RosSubscriber`
- Attach the `RosSubscriberExample` script to the `RosSubscriber` GameObject, drag the cube GameObject onto the `cube` parameter in the Inspector window and the `RosConnection` object onto the `ros` parameter.
- Press play in the editor

### In ROS Terminal Window
- After the scene has entered Play mode, run the following command: `rosrun robotics_demo color_publisher.py` to change the color of the cube GameObject in Unity to a random color

![](images/tcp_2.gif)

Continue to the [ROS–Unity Integration Service](service.md).