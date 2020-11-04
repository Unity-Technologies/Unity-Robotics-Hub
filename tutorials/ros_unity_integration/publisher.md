# Unity ROS Integration Publisher

Create a simple Unity scene which publishes a game object's position and rotation to a [ROS topic](http://wiki.ros.org/ROS/Tutorials/UnderstandingTopics#ROS_Topics).

## Setting Up ROS
- Follow the [Unity ROS Initial Setup](setup.md) guide.

- Now that your ROS endpoint is running, open a new terminal window and run the following command:
	- `rostopic echo pos_rot`

## Setting Up Unity Scene
- In the menu bar find and select `RosMessageGeneration` -> `Auto Generate Messages` -> `Single Message ...`
- Set the input file path to `PATH/TO/Unity-Robotics-Hub/tutorials/ros_packages/robotics_demo/msg/PosRot.msg ` and click `GENERATE!`
    - The generated file will be saved in the default directory `Assets/RosMessages/msg`
- Create a new directory in `Assets` and name it `Scripts`
- Create a new script in the `Scripts` directory and name it `RosPublisherExample.cs`
- Open `RosPublisherExample.cs` and paste the following code:
	- **Note** Script can be found at `tutorials/ros_unity_integration/unity_scripts`

```csharp
using RosMessageTypes.RoboticsDemo;
using UnityEngine;
using Random = UnityEngine.Random;

/// <summary>
/// 
/// </summary>
public class RosPublisherExample : MonoBehaviour
{
    public ROSConnection ros;
    public string topicName = "pos_rot";

    // The game object 
    public GameObject cube;
    // Publish the cube's position and rotation every N seconds
    public float publishMessageFrequency = 0.5f;

    // Used to determine how much time has elapsed since the last message was published
    private float timeElapsed;

    private void Update()
    {
        timeElapsed += Time.deltaTime;

        if (timeElapsed > publishMessageFrequency)
        {
            cube.transform.rotation = Random.rotation;

            PosRot cubePos = new PosRot(
                cube.transform.position.x,
                cube.transform.position.y,
                cube.transform.position.z,
                cube.transform.rotation.x,
                cube.transform.rotation.y,
                cube.transform.rotation.z,
                cube.transform.rotation.w
            );

            // Finally send the message to server_endpoint.py running in ROS
            ros.Send(topicName, cubePos);

            timeElapsed = 0;
        }
    }
}
```

- Add a plane and a cube to the empty Unity scene
- Move the cube a little ways up so it is hovering above the plane
- Create an empty game object, name it `RosConnection` and attach the `Plugins/TcpConnector/ROSConnection` script.
	- Change the host name and port to match the ROS IP and port variables defined when you set up ROS
- Create another empty game object, name it `RosPublisher` and attach the `RosPublisherExample` script.
	- Drag the cube game object onto the `Cube` parameter
	- Drag the RosConnection object onto its `Ros` parameter.

- Pressing play in the Editor should publish a message to the terminal running the `rostopic echo pos_rot` command every 0.5 seconds

![](images/tcp_1.gif)

Continue to the [ROS Subscriber](subscriber.md) tutorial.