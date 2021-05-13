# ROS–Unity Integration: Subscriber

Create a simple Unity scene which subscribes to a [ROS topic](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics#ROS_Topics) to change the colour of a GameObject.

## Setting Up ROS

(Skip to [Setting Up Unity Scene](subscriber.md#setting-up-unity-scene) if you already did the [Publisher](publisher.md) tutorial.)

- Copy the `tutorials/ros_packages/robotics_demo` folder of this repo into the `src` folder in your Catkin workspace.

- Follow the [ROS–Unity Initial Setup](setup.md) guide.

- Open a new terminal window, navigate to your Catkin workspace, and run the following commands:
  
   ```bash
    source devel/setup.bash
    rosrun robotics_demo server_endpoint.py
   ```

Once the server_endpoint has started, it will print something similar to `[INFO] [1603488341.950794]: Starting server on 192.168.50.149:10000`.

- In Unity, we need to generate the C# code for the `UnityColor` message. Open `Robotics` -> `Generate ROS Messages...`.
    - Set the ROS message path to `PATH/TO/Unity-Robotics-Hub/tutorials/ros_packages/robotics_demo/`, expand the robotics_demo subfolder and click `Build 2 msgs`.
    
![](images/generate_messages_1.png)

   - The generated files will be saved in the default directory `Assets/RosMessages/RoboticsDemo/msg`.

## Setting Up Unity Scene
- Create a script and name it `RosSubscriberExample.cs`
- Paste the following code into `RosSubscriberExample.cs`
    - **Note** Script can be found at `tutorials/ros_unity_integration/unity_scripts`

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosColor = RosMessageTypes.RoboticsDemo.MUnityColor;

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
- Press play in the editor

### In ROS Terminal Window
- After the scene has entered Play mode, run the following command: `rosrun robotics_demo color_publisher.py` to change the color of the cube GameObject in Unity to a random color

> Please reference [networking troubleshooting](network.md) doc if any errors are thrown.

![](images/tcp_2.gif)

## ROSSubscriberExample script

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosColor = RosMessageTypes.RoboticsDemo.MUnityColor;

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

### Import Statements for Subscriber and Messages 

```csharp
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using RosColor = RosMessageTypes.RoboticsDemo.MUnityColor;
```

### Instantiating the ROS Subscriber in unity

```csharp
void Start()
    {
        ROSConnection.instance.Subscribe<RosColor>("color", ColorChange);
    }
```
We declare the topic to be subscribed before the program execution starts. This is done in the `Start()` function of the MonoBehavior. We add the topic we want to subscribe to the internal dictionary of our ROSConnection instance using `Subscribe` API.

### Subscribe API
```csharp
public void Subscribe<T>(string topic, Action<T> callback)
```
### Summary
Its a non-static member of ROSConnection class. It is used to add a ROS subscriber to the internal ROSConnection Dictionary.
### Type Parameter
T: ROS message type published in the topic to be subscribed.
### Parameters
 Parameters  | Description |
| ------------- | ------------- |
| `string topic`  | Name of the topic to be subscribed on the ROS machine  |
| `Action<T> callback`  | Callback function used to process the received message |

### Implementing the callback function

```csharp
void ColorChange(RosColor colorMessage)
    {
        cube.GetComponent<Renderer>().material.color = new Color32((byte)colorMessage.r, (byte)colorMessage.g, (byte)colorMessage.b, (byte)colorMessage.a);
    }
```

After a message is received in Unity, a callback function is called to process the incoming message. The callback is a delegate which defines a function that receives one parameter and returns nothing as described [here](https://docs.microsoft.com/en-us/dotnet/api/system.action-1?view=net-5.0). In this Subscribe API, that parameter is the message published in the topic. 
In this example we extract the color's attributes from the message and assign it to the cube's material.

> Please reference [networking troubleshooting](network.md) doc if any errors are thrown.

![](images/tcp_2.gif)


Continue to the [ROS–Unity Integration Service](service.md).