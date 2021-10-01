# ROS–Unity Integration: UnityService

Create a simple Unity scene which runs a [Service](http://wiki.ros.org/Services) in Unity that takes a request with a GameObject's name and responds with the GameObject's pose (position and orientation) in the ROS coordinate system.

## Setting Up

- Follow the [ROS–Unity Demo Setup](setup.md) guide if you haven't already done so.

## Create Unity Service
- Create a new C# script and name it `RosUnityServiceExample.cs`
- Paste the following code into `RosUnityServiceExample.cs`
    - (Alternatively, you can drag the script file into Unity from `tutorials/ros_unity_integration/unity_scripts`).

```csharp
using RosMessageTypes.UnityRoboticsDemo;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
/// Example demonstration of implementing a UnityService that receives a Request message from another ROS node and sends a Response back
/// </summary>
public class RosUnityServiceExample : MonoBehaviour
{
    [SerializeField]
    string m_ServiceName = "obj_pose_srv";

    void Start()
    {
        // register the service with ROS
        ROSConnection.GetOrCreateInstance().ImplementService<ObjectPoseServiceRequest, ObjectPoseServiceResponse>(m_ServiceName, GetObjectPose);
    }

    /// <summary>
    ///  Callback to respond to the request
    /// </summary>
    /// <param name="request">service request containing the object name</param>
    /// <returns>service response containing the object pose (or 0 if object not found)</returns>
    private ObjectPoseServiceResponse GetObjectPose(ObjectPoseServiceRequest request)
    {
        // process the service request
        Debug.Log("Received request for object: " + request.object_name);

        // prepare a response
        ObjectPoseServiceResponse objectPoseResponse = new ObjectPoseServiceResponse();
        // Find a game object with the requested name
        GameObject gameObject = GameObject.Find(request.object_name);
        if (gameObject)
        {
            // Fill-in the response with the object pose converted from Unity coordinate to ROS coordinate system
            objectPoseResponse.object_pose.position = gameObject.transform.position.To<FLU>();
            objectPoseResponse.object_pose.orientation = gameObject.transform.rotation.To<FLU>();
        }

        return objectPoseResponse;
    }
}
```

- Create an empty GameObject and name it `UnityService`.
- Attach the `RosUnityServiceExample` script to the `UnityService` GameObject.
- Pressing play in the Editor should start running as a ROS node, waiting to accept ObjectPose requests. Once a connection to ROS has been established, a message will be printed on the ROS terminal similar to `Connection from 172.17.0.1`.


## Start the Client

- To test our new service is working, let's call it using the built-in ROS service command.

   a) <img src="images/ros1_icon.png" alt="ros1" width="14" height="14"/> In ROS1, run the following command in your ROS terminal:

   ```bash
   rosservice call /obj_pose_srv Cube
   ```

   In your Unity console you should see the log message `Received request for object: Cube`, and in your terminal it will report the object's position, like this:

   ```bash
   object_pose:
    position:
      x: 0.0
      y: -1.0
      z: 0.20000000298023224
    orientation:
      x: 0.0
      y: -0.0
      z: 0.0
      w: -1.0
   ```

   b) <img src="images/ros2_icon.png" alt="ros2" width="23" height="14"/> If you're using ROS2, the command is:
    ```bash
    ros2 service call obj_pose_srv unity_robotics_demo_msgs/ObjectPoseService "{object_name: Cube}"
	```

	And the output will look like this:

    ```bash
    requester: making request: unity_robotics_demo_msgs.srv.ObjectPoseService_Request(object_name='Cube')
    response:
	unity_robotics_demo_msgs.srv.ObjectPoseService_Response(object_pose=geometry_msgs.msg.Pose(position=geometry_msgs.msg.Point(x=0.0, y=-0.0, z=0.0), orientation=geometry_msgs.msg.Quaternion(x=-0.558996319770813, y=-0.3232670724391937, z=-0.6114855408668518, w=-0.4572822153568268)))
    ```

Continue to the [ROS–Unity Integration Service Call](service_call.md).