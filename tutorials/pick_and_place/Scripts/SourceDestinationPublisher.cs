using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using ROSGeometry;
using Quaternion = UnityEngine.Quaternion;

public class SourceDestinationPublisher : MonoBehaviour
{
    // ROS Connector
    public ROSConnection ros;
    private int numRobotJoints = 6;
    
    // Variables required for ROS communication
    public string topicName = "SourceDestination_input";

    public GameObject niryoOne;
    public GameObject target;
    public GameObject targetPlacement;
    
    private readonly Quaternion pickOrientation = new Quaternion(-0.5f,-0.5f,0.5f,-0.5f);
    
    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    
    /// <summary>
    /// 
    /// </summary>
    void Awake()
    {
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
        NiryoMoveitJoints sourceDestinationMessage = new NiryoMoveitJoints();

        sourceDestinationMessage.joint_00 = jointArticulationBodies[0].xDrive.target;
        sourceDestinationMessage.joint_01 = jointArticulationBodies[1].xDrive.target;
        sourceDestinationMessage.joint_02 = jointArticulationBodies[2].xDrive.target;
        sourceDestinationMessage.joint_03 = jointArticulationBodies[3].xDrive.target;
        sourceDestinationMessage.joint_04 = jointArticulationBodies[4].xDrive.target;
        sourceDestinationMessage.joint_05 = jointArticulationBodies[5].xDrive.target;

        // Pick Pose
        sourceDestinationMessage.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = target.transform.position.To<FLU>(),
            orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = targetPlacement.transform.position.To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };

        // Finally send the message to server_endpoint.py running in ROS
        ros.Send(topicName, sourceDestinationMessage);
    }
}
