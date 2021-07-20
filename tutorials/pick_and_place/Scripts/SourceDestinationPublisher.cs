using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
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

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;

    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;

        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = niryoOne.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string arm_link = shoulder_link + "/arm_link";
        jointArticulationBodies[1] = niryoOne.transform.Find(arm_link).GetComponent<ArticulationBody>();

        string elbow_link = arm_link + "/elbow_link";
        jointArticulationBodies[2] = niryoOne.transform.Find(elbow_link).GetComponent<ArticulationBody>();

        string forearm_link = elbow_link + "/forearm_link";
        jointArticulationBodies[3] = niryoOne.transform.Find(forearm_link).GetComponent<ArticulationBody>();

        string wrist_link = forearm_link + "/wrist_link";
        jointArticulationBodies[4] = niryoOne.transform.Find(wrist_link).GetComponent<ArticulationBody>();

        string hand_link = wrist_link + "/hand_link";
        jointArticulationBodies[5] = niryoOne.transform.Find(hand_link).GetComponent<ArticulationBody>();
    }

    public void Publish()
    {
        NiryoMoveitJointsMsg sourceDestinationMessage = new NiryoMoveitJointsMsg();

        sourceDestinationMessage.joint_00 = jointArticulationBodies[0].jointPositions[0] * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_01 = jointArticulationBodies[1].jointPositions[0] * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_02 = jointArticulationBodies[2].jointPositions[0] * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_03 = jointArticulationBodies[3].jointPositions[0] * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_04 = jointArticulationBodies[4].jointPositions[0] * Mathf.Rad2Deg;
        sourceDestinationMessage.joint_05 = jointArticulationBodies[5].jointPositions[0] * Mathf.Rad2Deg;

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
