using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using Unity.Robotics.UrdfImporter;
using RosMessageTypes.Geometry;

public class SourceDestinationPublisher : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Variables required for ROS communication
    public string topicName = "SourceDestination_input";

    public GameObject niryoOne;
    public GameObject target;
    public GameObject targetPlacement;

    private int numRobotJoints = 6;
    private readonly Quaternion pickOrientation = Quaternion.Euler(90, 90, 0);

    // Robot Joints
    private UrdfJointRevolute[] revoluteJoints;

    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        revoluteJoints = new UrdfJointRevolute[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        revoluteJoints[0] = niryoOne.transform.Find(shoulder_link).GetComponent<UrdfJointRevolute>();

        string arm_link = shoulder_link + "/arm_link";
        revoluteJoints[1] = niryoOne.transform.Find(arm_link).GetComponent<UrdfJointRevolute>();

        string elbow_link = arm_link + "/elbow_link";
        revoluteJoints[2] = niryoOne.transform.Find(elbow_link).GetComponent<UrdfJointRevolute>();

        string forearm_link = elbow_link + "/forearm_link";
        revoluteJoints[3] = niryoOne.transform.Find(forearm_link).GetComponent<UrdfJointRevolute>();

        string wrist_link = forearm_link + "/wrist_link";
        revoluteJoints[4] = niryoOne.transform.Find(wrist_link).GetComponent<UrdfJointRevolute>();

        string hand_link = wrist_link + "/hand_link";
        revoluteJoints[5] = niryoOne.transform.Find(hand_link).GetComponent<UrdfJointRevolute>();
    }

    public void Publish()
    {
        NiryoMoveitJointsMsg sourceDestinationMessage = new NiryoMoveitJointsMsg();

        sourceDestinationMessage.joint_00 = revoluteJoints[0].GetPosition() * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_01 = revoluteJoints[1].GetPosition() * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_02 = revoluteJoints[2].GetPosition() * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_03 = revoluteJoints[3].GetPosition() * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_04 = revoluteJoints[4].GetPosition() * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_05 = revoluteJoints[5].GetPosition() * Mathf.Rad2Deg;

        // Pick Pose
        sourceDestinationMessage.pick_pose = new PoseMsg
        {
            position = target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new PoseMsg
        {
            position = targetPlacement.transform.position.To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        ros.Send(topicName, sourceDestinationMessage);
    }
}
