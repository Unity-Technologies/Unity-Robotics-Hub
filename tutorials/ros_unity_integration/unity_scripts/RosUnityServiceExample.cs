using RosMessageTypes.RoboticsDemo;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;

/// <summary>
/// Example demonstration of implementing a UnityService that receives a Request message from another ROS node and sends a Response back
/// </summary>
public class RosUnityServiceExample : MonoBehaviour
{
    public string serviceName = "obj_pose_srv";

    void Start()
    {
        // register the service with ros
        ROSConnection.instance.ImplementService<MObjectPoseServiceRequest>(serviceName, GetObjectPose);
    }

    /// <summary>
    ///  Callback to respond to the request
    /// </summary>
    /// <param name="request">service request containing the object name</param>
    /// <returns>service response containing the object pose (or 0 if object not found)</returns>
    private MObjectPoseServiceResponse GetObjectPose(MObjectPoseServiceRequest request)
    {
        // process the service request
        Debug.Log("Received request for object: " + request.object_name);

        // prepare a response
        MObjectPoseServiceResponse objectPoseResponse = new MObjectPoseServiceResponse();
        GameObject gameObject = GameObject.Find(request.object_name);
        if (gameObject) 
        {
            objectPoseResponse.object_pose.position = gameObject.transform.position.To<FLU>();
            objectPoseResponse.object_pose.orientation = gameObject.transform.rotation.To<FLU>();
        }
       
        return objectPoseResponse;
    }
}
