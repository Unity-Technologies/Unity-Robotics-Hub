# Unity ROS Integration Subscriber

Create a simple Unity scene which subscribes to a [ROS topic](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics#ROS_Topics) to change the colour of a game object.

**NOTE:** If following from [Publisher](publisher.md) tutorial proceed to [Setting Up Unity Scene](subscriber.md#setting-up-unity-scene) step.

Follow the [Initial ROS Setup](setup.md) guide.

Follow the [ROS Message Generation](https://github.com/Unity-Technologies/Unity-Robotics-Hub/blob/master/tutorials/unity_ros_message_generation/message_generation_tutorial.md) guide.

Follow the [Setting Up ROS](ros_setup.md) guide.

## Setting Up Unity Scene
- Generate the C# code for `UnityColor` message by going to `RosMessageGeneration` -> `AutoGenerateMessages` -> `Single Message...`
- Set the input file path to `PATH/TO/Unity-Robotics-Hub/tutorials/ros_packages/robotics_demo/msg/UnityColor.msg` and click `GENERATE!`
    - The generated file will be saved in the default directory `Assets/RosMessages/msg`
- Creat a script and name it `RosSubscriberExample.cs`
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
        StartMessageServer(hostPort);
    }

}
```

- Create an empty game object and name it `RosSubscriber`
- Attach the `RosSubscriberExample` script to the `RosSubscriber` game object and drag the cube game object onto the `cube` parameter ni the Inspector window.
- In the Inspector window of the Editor change the `hostName` parameter on the `RosSubscriber ` game object to the ROS master URI. 
- Press play in the editor

### In ROS Terminal Window
- After the scene has entered Play mode, run the following command: `rosrun robotics_demo color_publisher.py` to change the color of the cube game object in Unity to a random color

![](images/tcp_2.gif)

Continue to the [Unity ROS Integration Service](service.md).