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
    
    private readonly RosQuaternion pickOrientation = new RosQuaternion(0.5,0.5,-0.5,0.5);
    
    
    // Start is called before the first frame update
    void Start()
    {
        // Instantiate the connector with ROS host name and port.
        tcpCon = new TcpConnector(hostName, hostPort);
    }

    // Update is called once per frame
    public void Publish()
    {
        NiryoMoveitJoints sourceDestinationMessage = new NiryoMoveitJoints();

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
