using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;

public class SourceDestinationPublisher : MonoBehaviour
{
    private TcpConnector tcpCon;
    
    // Variables required for ROS communication
    public string topicName = "SourceDestination_input";
    public string hostName = "192.168.50.149";
    public int hostPort = 10000;

    public GameObject target;
    public GameObject targetPlacement;
    public GameObject[] jointGameObjects;
    
    private readonly RosQuaternion pickOrientation = new RosQuaternion(0.5,0.5,-0.5,0.5);
    private ArticulationBody[] jointArticulationBodies;
    
    // Start is called before the first frame update
    void Start()
    {
        // Instantiate the connector with ROS host name and port.
        tcpCon = new TcpConnector(hostName, hostPort);

        jointArticulationBodies = new ArticulationBody[jointGameObjects.Length];
        // Setup articulation bodies
        for (int i = 0; i < jointGameObjects.Length; i++)
        {
            jointArticulationBodies[i] = jointGameObjects[i].GetComponent<ArticulationBody>();
        }
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
            position = new Point(
                target.transform.position.z,
                -target.transform.position.x,
                target.transform.position.y
            ),
            orientation = pickOrientation
        };

        // Place Pose
        sourceDestinationMessage.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = new Point(
                targetPlacement.transform.position.z,
                -targetPlacement.transform.position.x,
                targetPlacement.transform.position.y
            ),
            orientation = pickOrientation
        };

        // Finally send the message to server_endpoint.py running in ROS
        tcpCon.SendMessage(topicName, sourceDestinationMessage);
    }
}
