# ROS-Unity Integration Subscriber

Create a simple Unity scene which subscribes to a [ROS topic](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics#ROS_Topics) to change the colour of a game object.

**NOTE:** If following from [Publisher](publisher.md) tutorial proceed to [Setting Up Unity Scene](subscriber.md#setting-up-unity-scene) step.

## Setting Up ROS
(You can skip this if you already did the [ROS-Unity Integration Publisher](publisher.md) tutorial.)

- Copy the `tutorials/ros_packages/robotics_demo` folder of this repo into the `src` folder in your Catkin workspace.

- Follow the [ROS-Unity Initial Setup](setup.md) guide.

- Open a new terminal window and run the following commands:
  
   ```bash
    cd ~/catkin_ws/ && source devel/setup.bash
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
using System.Net.Sockets;
using System.Threading.Tasks;
using UnityEngine;
using RosColor = RosMessageTypes.RoboticsDemo.UnityColor;

public class RosSubscriberExample : RosSubscriber
{
    public GameObject cube;

    protected override async Task HandleConnectionAsync(TcpClient tcpClient)
    {
        await Task.Yield();

        using (var networkStream = tcpClient.GetStream())
        {
            RosColor colorMessage = (RosColor)ReadMessage(networkStream, new RosColor());
            Debug.Log("Color(" + colorMessage.r + ", "+ colorMessage.g + ", "+ colorMessage.b + ", "+ colorMessage.a +")");
            cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
        }
    }

    void Start()
    {
        StartMessageServer();
    }

}
```

- Create an empty game object and name it `RosSubscriber`
- Attach the `RosSubscriberExample` script to the `RosSubscriber` game object and drag the cube game object onto the `cube` parameter in the Inspector window.
- In the Inspector window of the Editor change the `unityHostName` parameter on the `RosSubscriber ` game object to the Unity machine's URI. (Note for Windows users: the connection will be rejected unless you put the actual IP address ROS is connecting to. That's not necessarily the same as your machine's main IP address.)
- Press play in the editor

### In ROS Terminal Window
- After the scene has entered Play mode, run the following command: `rosrun robotics_demo color_publisher.py` to change the color of the cube game object in Unity to a random color

![](images/tcp_2.gif)

Continue to the [ROS-Unity Integration Service](service.md).