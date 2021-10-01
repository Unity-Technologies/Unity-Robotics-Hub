# ROS–Unity Integration: Service Call

Create a simple Unity scene which calls an external [ROS service](http://wiki.ros.org/Services) with a GameObject's position and rotation to receive a new position to move the GameObject towards.

## Setting Up

- Follow the [ROS–Unity Demo Setup](setup.md) guide if you haven't already done so.

## Start the Position service
- For this tutorial we will need a ros service for Unity to call. In a new terminal window, navigate to your ROS workspace.

   a) <img src="images/ros1_icon.png" alt="ros1" width="14" height="14"/> In ROS1, run the following commands:

   ```bash
    source devel/setup.bash
    rosrun unity_robotics_demo position_service.py
   ```

   b) <img src="images/ros2_icon.png" alt="ros2" width="23" height="14"/> In ROS2, instead run:

     ```bash
      source install/setup.bash
      ros2 run unity_robotics_demo position_service
     ```


## Create Unity Service Caller
- Create a script and name it `RosServiceCallExample.cs`
- Paste the following code into `RosServiceCallExample.cs`
    - (Alternatively, you can drag the script file into Unity from `tutorials/ros_unity_integration/unity_scripts`).

```csharp
using RosMessageTypes.UnityRoboticsDemo;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;

public class RosServiceCallExample : MonoBehaviour
{
    ROSConnection ros;

    public string serviceName = "pos_srv";

    public GameObject cube;

    // Cube movement conditions
    public float delta = 1.0f;
    public float speed = 2.0f;
    private Vector3 destination;

    float awaitingResponseUntilTimestamp = -1;

    void Start()
    {
        ros = ROSConnection.GetOrCreateInstance();
        ros.RegisterRosService<PositionServiceRequest, PositionServiceResponse>(serviceName);
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

            PosRotMsg cubePos = new PosRotMsg(
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
            awaitingResponseUntilTimestamp = Time.time + 1.0f; // don't send again for 1 second, or until we receive a response
        }
    }

    void Callback_Destination(PositionServiceResponse response)
    {
        awaitingResponseUntilTimestamp = -1;
        destination = new Vector3(response.output.pos_x, response.output.pos_y, response.output.pos_z);
        Debug.Log("New Destination: " + destination);
    }
}
```

- Create an empty GameObject and name it `RosService`.
- Attach the `RosServiceExample` script to the `RosService` GameObject. Drag the cube GameObject onto its `cube` parameter.
- Pressing play in the Editor should start communication with the `position_service` script, running as a ROS node, causing the cube to move to random positions in the scene.


> Please reference [networking troubleshooting](network.md) doc if any errors are thrown.

![](images/tcp_3.gif)
