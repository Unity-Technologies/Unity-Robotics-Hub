using System.Collections;
using System.Linq;
using System.Net.Sockets;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoMoveit;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;

public class TrajectoryPlanner : MonoBehaviour
{
    // ROS Connector
    private TcpConnector tcpCon;
    private int numRobotJoints = 6;

    // Hardcoded variables 
    private readonly float jointAssignmentWait = 0.1f;
    private readonly float poseAssignmentWait = 0.5f;
    private readonly float pickPoseOffset = 0.1f;
    
    // Assures that the gripper is always positioned above the target cube before grasping.
    private readonly RosQuaternion pickOrientation = new RosQuaternion(0.5,0.5,-0.5,0.5);

    // Variables required for ROS communication
    public string rosServiceName = "niryo_moveit";
    public string hostName = "192.168.50.149";
    public int hostPort = 10000;
    public int connectionTimeout = 10;

    public GameObject niryoOne;
    public GameObject target;
    public GameObject targetPlacement;

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
    
    /// <summary>
    /// 
    /// </summary>
    private void CloseGripper()
    {
        var leftDrive = leftGripper.xDrive;
        var rightDrive = rightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        leftGripper.xDrive = leftDrive;
        rightGripper.xDrive = rightDrive;
    }

    /// <summary>
    /// 
    /// </summary>
    private void OpenGripper()
    {
        var leftDrive = leftGripper.xDrive;
        var rightDrive = rightGripper.xDrive;

        leftDrive.target = 0;
        rightDrive.target = 0;

        leftGripper.xDrive = leftDrive;
        rightGripper.xDrive = rightDrive;
    }

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
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

    /// <summary>
    /// 
    /// </summary>
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
                // Add pick pose offset to position the gripper above target to avoid collisions
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
            StartCoroutine(ExecuteTrajectories(response));
        }
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="response"></param>
    /// <returns></returns>
    private IEnumerator ExecuteTrajectories(MoverServiceResponse response)
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

        // Find left and right fingers
        string right_gripper = hand_link + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        string left_gripper = hand_link + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";
        leftGripper = niryoOne.transform.Find(left_gripper).GetComponent<ArticulationBody>();
        rightGripper = niryoOne.transform.Find(right_gripper).GetComponent<ArticulationBody>();
    }

    /// <summary>
    /// 
    /// </summary>
    void Start()
    {
        // Instantiate the connector with ROS host name and port.
        tcpCon = new TcpConnector(hostName, hostPort, serviceResponseRetry: 10, serviceResponseSleep: 1000);
    }
}