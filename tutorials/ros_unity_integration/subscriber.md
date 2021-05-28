# ROS–Unity Integration: Subscriber

Create a simple Unity scene which subscribes to a [ROS topic](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics#ROS_Topics) to change the colour of a GameObject.

## Setting Up ROS

(Skip to [Setting Up Unity Scene](subscriber.md#setting-up-unity-scene) if you already did the [Publisher](publisher.md) tutorial.)

- Copy the `tutorials/ros_unity_integration/ros_packages/robotics_demo` folder of this repo into the `src` folder in your Catkin workspace.

- Follow the [ROS–Unity Initial Setup](setup.md) guide.

- Open a new terminal window, navigate to your Catkin workspace, and run the following commands:

   ```bash
    source devel/setup.bash
    rosrun robotics_demo server_endpoint.py
   ```

Once the server_endpoint has started, it will print something similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000`.

## Setting Up ROS2

(Skip to [Setting Up Unity Scene](subscriber.md#setting-up-unity-scene) if you already did the [Publisher](publisher.md) tutorial.)

- Copy the `tutorials/ros_unity_integration/ros2_packages/robotics_demo` folder of this repo into the `src` folder in your Colcon workspace.

- Follow the [ROS–Unity Initial Setup](setup.md) guide.

- Open a new terminal window, navigate to your Colcon workspace, and run the following commands:

   ```bash
    source install/setup.bash
    ros2 run robotics_demo server_endpoint.py
   ```

Once the server_endpoint has started, it will print something similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000`.

## Setting Up Unity Scene

- In Unity, we need to generate the C# code for the `UnityColor` message. Open `Robotics` -> `Generate ROS Messages...`.
    - Set the ROS message path to `PATH/TO/Unity-Robotics-Hub/tutorials/ros_unity_integration/ros_packages/robotics_demo/`, expand the robotics_demo subfolder and click `Build 2 msgs`.

![](images/generate_messages_1.png)

   - The generated files will be saved in the default directory `Assets/RosMessages/RoboticsDemo/msg`.

- Create a script and name it `RosSubscriberExample.cs`
- Paste the following code into `RosSubscriberExample.cs`
    - Alternatively, copy the script file from `tutorials/ros_unity_integration/unity_scripts/RosSubscriberExample.cs`

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosColor = RosMessageTypes.UnityRoboticsDemo.UnityColorMsg;

public class RosSubscriberExample : MonoBehaviour
{
    public GameObject cube;

    void Start()
    {
        ROSConnection.instance.Subscribe<RosColor>("color", ColorChange);
    }

    void ColorChange(RosColor colorMessage)
    {
        cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
    }
}
```

- Create an empty GameObject and name it `RosSubscriber`
- Attach the `RosSubscriberExample` script to the `RosSubscriber` GameObject and drag the cube GameObject onto the `cube` parameter in the Inspector window.

- From the Unity menu bar, open `Robotics/ROS Settings`, and set the `ROS IP Address` variable to your ROS IP.
	- If using ROS2, also switch the protocol to ROS2.
- Press play in the editor

### In ROS Terminal Window
- After the scene has entered Play mode, run the following command: `rosrun robotics_demo color_publisher.py` to change the color of the cube GameObject in Unity to a random color.

- In ROS2, instead run `ros2 run unity_robotics_demo color_publisher`.

> Please reference [networking troubleshooting](network.md) doc if any errors are thrown.

![](images/tcp_2.gif)

Continue to the [ROS–Unity Integration Service](service.md).