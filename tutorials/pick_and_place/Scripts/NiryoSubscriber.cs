using System.Collections;
using System.Collections.Generic;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.NiryoMoveit;
using RosMessageTypes.NiryoOne;
using UnityEngine;
using RosQuaternion = RosMessageTypes.Geometry.Quaternion;
using Transform = UnityEngine.Transform;

public class NiryoSubscriber : MonoBehaviour
{
    private const int OPEN_GRIPPER = 2;
    private const int CLOSE_GRIPPER = 1;
    private const int TOOL_COMMAND_EXECUTION = 6;
    private const int TRAJECTORY_COMMAND_EXECUTION = 7;
    
    public ROSConnection ros;
    private const int NUM_ROBOT_JOINTS = 6;
    
    // Hardcoded variables 
    private const float JOINT_ASSIGNMENT_WAIT = 0.1f;
    private const float PICK_POSE_OFFSET = 0.1f;
    
    // Assures that the gripper is always positioned above the target cube before grasping.
    private readonly RosQuaternion pickOrientation = new RosQuaternion(0.5,0.5,-0.5,0.5);

    // Variables required for ROS communication
    public string rosJointPublishTopicName = "publish_target";
    public string rosRobotCommandsTopicName = "robot_command";

    public GameObject niryoOne;
    public GameObject target;
    public GameObject targetPlacement;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    private ArticulationBody leftGripperJoint;
    private ArticulationBody rightGripperJoint;

    private Transform leftGripperGameObject;
    private Transform rightGripperGameObject;

    private readonly List<RobotMoveActionGoal> moveActionGoals = new List<RobotMoveActionGoal>();
    public float commandExecutionSleep = 0.5f;

    /// <summary>
    ///     Close the gripper
    /// </summary>
    private void CloseGripper()
    {
        var leftDrive = leftGripperJoint.xDrive;
        var rightDrive = rightGripperJoint.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        leftGripperJoint.xDrive = leftDrive;
        rightGripperJoint.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    private void OpenGripper()
    {
        var leftDrive = leftGripperJoint.xDrive;
        var rightDrive = rightGripperJoint.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        leftGripperJoint.xDrive = leftDrive;
        rightGripperJoint.xDrive = rightDrive;
    }


    /// <summary>
    /// 
    /// </summary>
    public void PublishJoints()
    {
        MoverServiceRequest request = new MoverServiceRequest
        {
            joints_input =
            {
                joint_00 = jointArticulationBodies[0].xDrive.target,
                joint_01 = jointArticulationBodies[1].xDrive.target,
                joint_02 = jointArticulationBodies[2].xDrive.target,
                joint_03 = jointArticulationBodies[3].xDrive.target,
                joint_04 = jointArticulationBodies[4].xDrive.target,
                joint_05 = jointArticulationBodies[5].xDrive.target
            },
            pick_pose = new RosMessageTypes.Geometry.Pose
            {

                position = (target.transform.position + pickPoseOffset).To<FLU>(),,
                // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
                orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
            },
            place_pose = new RosMessageTypes.Geometry.Pose
            {
                position = (targetPlacement.transform.position + pickPoseOffset).To<FLU>(),
                orientation = pickOrientation.To<FLU>()
            }
        };
        
        ros.Send(rosJointPublishTopicName, request);
    }
    
    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Awake()
    {
        jointArticulationBodies = new ArticulationBody[NUM_ROBOT_JOINTS];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = niryoOne.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string armLink = shoulder_link + "/arm_link";
        jointArticulationBodies[1] = niryoOne.transform.Find(armLink).GetComponent<ArticulationBody>();
        
        string elbowLink = armLink + "/elbow_link";
        jointArticulationBodies[2] = niryoOne.transform.Find(elbowLink).GetComponent<ArticulationBody>();
        
        string forearmLink = elbowLink + "/forearm_link";
        jointArticulationBodies[3] = niryoOne.transform.Find(forearmLink).GetComponent<ArticulationBody>();
        
        string wristLink = forearmLink + "/wrist_link";
        jointArticulationBodies[4] = niryoOne.transform.Find(wristLink).GetComponent<ArticulationBody>();
        
        string handLink = wristLink + "/hand_link";
        jointArticulationBodies[5] = niryoOne.transform.Find(handLink).GetComponent<ArticulationBody>();

        // Find left and right fingers
        string rightGripper = handLink + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        string leftGripper = handLink + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        leftGripperGameObject = niryoOne.transform.Find(leftGripper);
        rightGripperGameObject = niryoOne.transform.Find(rightGripper);

        rightGripperJoint = rightGripperGameObject.GetComponent<ArticulationBody>();
        leftGripperJoint = leftGripperGameObject.GetComponent<ArticulationBody>();
    }
    
    void Start()
    {
        ros.Subscribe<RobotMoveActionGoal>(rosRobotCommandsTopicName, AppendRobotCommands);
        StartCoroutine(ExecuteRobotCommand());
    }

    /// <summary>
    /// 
    /// </summary>
    /// <param name="robotAction"></param>
    void AppendRobotCommands(RobotMoveActionGoal robotAction)
    {
        moveActionGoals.Add(robotAction);
    }
    

    /// <summary>
    /// 
    /// </summary>
    /// <returns></returns>
    IEnumerator ExecuteRobotCommand()
    {
        for (;;)
        {
            if (moveActionGoals.Count > 0)
            {
                if (moveActionGoals[0].goal.cmd.cmd_type == TRAJECTORY_COMMAND_EXECUTION)
                {
                    StartCoroutine(ExecuteTrajectories(moveActionGoals[0].goal.cmd.Trajectory.trajectory));
                }
                else if (moveActionGoals[0].goal.cmd.cmd_type == TOOL_COMMAND_EXECUTION)
                {
                    if (moveActionGoals[0].goal.cmd.tool_cmd.cmd_type == OPEN_GRIPPER)
                    {
                        OpenGripper();
                    }
                    else if (moveActionGoals[0].goal.cmd.tool_cmd.cmd_type == CLOSE_GRIPPER)
                    {
                        CloseGripper();
                    }
                }
                // Remove executed command
                moveActionGoals.RemoveAt(0);
            }

            yield return new WaitForSeconds(commandExecutionSleep);
        }
    }

    /// <summary>
    ///     Execute the returned trajectories from the MoverService.
    /// 
    ///     The expectation is that the MoverService will return four trajectory plans,
    ///         PreGrasp, Grasp, PickUp, and Place,
    ///     where each plan is an array of robot poses. A robot pose is the joint angle values
    ///     of the six robot joints.
    /// 
    ///     Executing a single trajectory will iterate through every robot pose in the array while updating the
    ///     joint values on the robot.
    /// 
    /// </summary>
    /// <param name="trajectories"></param>
    private IEnumerator ExecuteTrajectories(RobotTrajectory trajectories)
    {
        // For every robot pose in trajectory plan
        foreach (var point in trajectories.joint_trajectory.points)
        {
            var jointPositions = point.positions;
            float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
            
            // Set the joint values for every joint
            for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            {
                var joint1XDrive  = jointArticulationBodies[joint].xDrive;
                joint1XDrive.target = result[joint];
                jointArticulationBodies[joint].xDrive = joint1XDrive;
            }
            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
        }
    }
}
