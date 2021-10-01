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
