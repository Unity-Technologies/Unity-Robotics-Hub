# ROS–Unity Integration Service

Create a simple Unity scene which calls a [ROS service](http://wiki.ros.org/Services) with a game object's position and rotation to receive a new position to move the game object towards.

## Setting Up ROS
- Follow the [ROS–Unity Initial Setup](setup.md) guide. (You can skip this step if following from the [ROS–Unity Integration Publisher](publisher.md) or [Subscriber](subscriber.md) tutorials.)
- Open a new terminal window and run `rosrun robotics_demo position_service.py`

## Setting Up Unity Scene
- Generate the C# code for `PositionService`'s messages by going to `RosMessageGeneration` -> `AutoGenerateServices` -> `Single Service...`
- Set the input file path to `PATH/TO/Unity-Robotics-Hub/tutorials/ros_packages/robotics_demo/srv/PositionService.srv` and click `GENERATE!`
    - The generated files will be saved in the default directory `Assets/RosMessages/srv`
- Create a script and name it `RosServiceExample.cs`
- Paste the following code into `RosServiceExample.cs`
- **Note** Script can be found at `tutorials/ros_unity_integration/unity_scripts`

```csharp
using System;
using RosMessageTypes.RoboticsDemo;
using UnityEngine;
using Debug = UnityEngine.Debug;

public class RosServiceExample : MonoBehaviour
{
    public ROSConnection ros;

    public string serviceName = "pos_srv";

    public GameObject cube;

    // Cube movement conditions
    public float delta = 1.0f;
    public float speed = 2.0f;
    private Vector3 destination;

    float awaitingResponseUntilTimestamp = -1;

    void Start()
    {
        destination = cube.transform.position;
    }

    private void Update()
    {
        // Move our position a step closer to the target.
        float step = speed * Time.deltaTime; // calculate distance to move
        cube.transform.position = Vector3.MoveTowards(cube.transform.position, destination, step);

        if (Vector3.Distance(cube.transform.position, destination) < delta && Time.time > awaitingResponseUntilTimestamp)
        {
            Debug.Log("Destination reached.");

            PosRot cubePos = new PosRot(
                cube.transform.position.x,
                cube.transform.position.y,
                cube.transform.position.z,
                cube.transform.rotation.x,
                cube.transform.rotation.y,
                cube.transform.rotation.z,
                cube.transform.rotation.w
            );

            PositionServiceRequest positionServiceRequest = new PositionServiceRequest(cubePos);

            // Send message to ROS and return the response
            ros.SendServiceMessage<PositionServiceResponse>(serviceName, positionServiceRequest, Callback_Destination);
            awaitingResponseUntilTimestamp = Time.time+1.0f; // don't send again for 1 second, or until we receive a response
        }
    }

    void Callback_Destination(PositionServiceResponse response)
    {
        awaitingResponseUntilTimestamp = -1;
        destination = new Vector3(response.output.pos_x, response.output.pos_y, response.output.@for);
        Debug.Log("New Destination: " + destination);
    }
}
```

- Create an empty game object, name it `RosConnection`, and attach the `Plugins/TCPConnection/ROSConnection` script to it.
- In the Inspector window of the Editor change the `hostName` variable on the `RosConnection` game object to the ROS IP.
- Create another empty game object and name it `RosService`.
- Attach the `RosServiceExample` script to the `RosService` game object. Drag the cube game object onto its `cube` parameter and the RosConnection game object onto its `Ros` parameter.
- Pressing play in the Editor should start communication with the `postion_service` script, running as a ROS node, causing the cube to move to random positions in the scene.

![](images/tcp_3.gif)
