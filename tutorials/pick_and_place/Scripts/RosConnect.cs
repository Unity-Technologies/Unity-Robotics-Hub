using System.Collections;
using System.Linq;
using System.Net.Sockets;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;

public class RosConnect : MonoBehaviour
{
    // ROS Connector
    private TcpConnector tcpCon;

    private readonly float jointAssignmentWait = 0.06f;
    private readonly float poseAssignmentWait = 1f;
    private readonly float pickPoseOffset = 0.08f;
    private readonly RosQuaternion pickOrientation = new RosQuaternion(0.5,0.5,-0.5,0.5);

    // Variables required for ROS communication
    public string rosServiceName = "niryo_moveit";
    public string hostName = "192.168.50.149";
    public int hostPort = 10000;
    public int connectionTimeout = 10;

    // The game objects
    public GameObject target;
    public GameObject targetPlacement;

    // GameObjects used to get Articulation Bodies
    public GameObject[] jointGameObjects;
    public GameObject leftGripperGO;
    public GameObject rightGripperGO;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    private ArticulationBody leftGripper;
    private ArticulationBody rightGripper;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place
    };
    
    private void CloseGripper()
    {
        var leftDrive = leftGripper.xDrive;
        var rightDrive = rightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        leftGripper.xDrive = leftDrive;
        rightGripper.xDrive = rightDrive;
    }

    private void OpenGripper()
    {
        var leftDrive = leftGripper.xDrive;
        var rightDrive = rightGripper.xDrive;

        leftDrive.target = 0;
        rightDrive.target = 0;

        leftGripper.xDrive = leftDrive;
        rightGripper.xDrive = rightDrive;
    }

    NiryoMoveitJoints CurrentJointConfig()
    {
        NiryoMoveitJoints joints = new NiryoMoveitJoints();
        
        joints.joint_00 = jointArticulationBodies[0].xDrive.target;
        joints.joint_01 = jointArticulationBodies[1].xDrive.target;
        joints.joint_02 = jointArticulationBodies[2].xDrive.target;
        joints.joint_03 = jointArticulationBodies[3].xDrive.target;
        joints.joint_04 = jointArticulationBodies[4].xDrive.target;
        joints.joint_05 = jointArticulationBodies[5].xDrive.target;

        return joints;
    }

    public void PublishJoints()
    {
        MoverServiceRequest request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();
        
        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = new Point(
                target.transform.position.z,
                -target.transform.position.x,
                target.transform.position.y + pickPoseOffset
            ),
            orientation = pickOrientation
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = new Point(
                targetPlacement.transform.position.z,
                -targetPlacement.transform.position.x,
                targetPlacement.transform.position.y + pickPoseOffset
            ),
            orientation = pickOrientation
        };

        var response = (MoverServiceResponse)tcpCon.SendServiceMessage(rosServiceName, request, new MoverServiceResponse());
        if (response.trajectories != null)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(PrintTrajectories(response));
        }
    }

    private IEnumerator PrintTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            for (int poseIndex  = 0 ; poseIndex < response.trajectories.Length; poseIndex++)
            {
                for (int jointConfigIndex  = 0 ; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
                    
                    for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive  = jointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        jointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    yield return new WaitForSeconds(jointAssignmentWait);
                }

                if (poseIndex == (int)Poses.Grasp)
                    CloseGripper();
                
                yield return new WaitForSeconds(poseAssignmentWait);
            }
            // Open Gripper at end of sequence
            OpenGripper();
        }
    }

    void Start()
    {
        TcpClient client = new TcpClient();

        // Instantiate the connector with ROS host name and port.
        tcpCon = new TcpConnector(hostName, hostPort, serviceResponseRetry: 10, serviceResponseSleep: 1000);
        
        jointArticulationBodies = new ArticulationBody[jointGameObjects.Length];
        // Setup articulation bodies
        for (int i = 0; i < jointGameObjects.Length; i++)
        {
            jointArticulationBodies[i] = jointGameObjects[i].GetComponent<ArticulationBody>();
        }

        leftGripper = leftGripperGO.GetComponent<ArticulationBody>();
        rightGripper = rightGripperGO.GetComponent<ArticulationBody>();
    }
}