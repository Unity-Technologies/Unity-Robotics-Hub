using System.Collections;
using System.Linq;
using RosMessageTypes.Geometry;
using RosMessageTypes.Moveit;
using RosMessageTypes.NiryoMoveit;
using RosMessageTypes.NiryoOne;
using Unity.Robotics.ROSTCPConnector;
using Unity.Robotics.ROSTCPConnector.ROSGeometry;
using UnityEngine;

public class RealSimPickAndPlace : MonoBehaviour
{
    private const int OPEN_GRIPPER = 1;
    private const int CLOSE_GRIPPER = 2;
    private const int TOOL_COMMAND_EXECUTION = 6;
    private const int TRAJECTORY_COMMAND_EXECUTION = 7;

    private ROSConnection ros;
    private const int NUM_ROBOT_JOINTS = 6;

    // Hardcoded variables 
    private const float JOINT_ASSIGNMENT_WAIT = 0.038f;
    private readonly Vector3 PICK_POSE_OFFSET = Vector3.up * 0.15f;

    // Assures that the gripper is always positioned above the target cube before grasping.
    private readonly Quaternion pickOrientation = Quaternion.Euler(90, 90, 0);

    // Variables required for ROS communication
    public string rosJointPublishTopicName = "sim_real_pnp";
    public string rosRobotCommandsTopicName = "niryo_one/commander/robot_action/goal";

    public GameObject niryoOne;
    public GameObject target;
    public GameObject targetPlacement;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    private ArticulationBody leftGripperJoint;
    private ArticulationBody rightGripperJoint;

    private Transform leftGripperGameObject;
    private Transform rightGripperGameObject;

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
    ///     Publish the robot's current joint configuration, the target object's
    ///     position and rotation, and the target placement for the target object's
    ///     position and rotation.
    ///
    ///     Includes conversion from Unity coordinates to ROS coordinates, Forward Left Up
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
            pick_pose = new PoseMsg
            {
                position = (target.transform.position + PICK_POSE_OFFSET).To<FLU>(),
                // The hardcoded x/z angles assure that the gripper is always positioned above the target cube before grasping.
                orientation = Quaternion.Euler(90, target.transform.eulerAngles.y, 0).To<FLU>()
            },
            place_pose = new PoseMsg
            {
                position = (targetPlacement.transform.position + PICK_POSE_OFFSET).To<FLU>(),
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
        ros.Subscribe<RobotMoveActionGoal>(rosRobotCommandsTopicName, ExecuteRobotCommands);
    }

    /// <summary>
    ///   Execute robot commands receved from ROS Subscriber.
    ///   Gripper commands will be executed immeditately wihle trajectories will be 
    ///   executed in a coroutine.
    /// </summary>
    /// <param name="robotAction"> RobotMoveActionGoal of trajectory or gripper commands</param>
    void ExecuteRobotCommands(RobotMoveActionGoal robotAction)
    {
        if (robotAction.goal.cmd.cmd_type == TRAJECTORY_COMMAND_EXECUTION)
        {
            StartCoroutine(ExecuteTrajectories(robotAction.goal.cmd.Trajectory.trajectory));
        }
        else if (robotAction.goal.cmd.cmd_type == TOOL_COMMAND_EXECUTION)
        {
            if (robotAction.goal.cmd.tool_cmd.cmd_type == OPEN_GRIPPER)
            {
                Debug.Log("Open Tool Command");
                OpenGripper();
            }
            else if (robotAction.goal.cmd.tool_cmd.cmd_type == CLOSE_GRIPPER)
            {
                Debug.Log("Close Tool Command");
                CloseGripper();
            }
        }
    }

    /// <summary>
    ///     Execute trajectories from RobotMoveActionGoal topic.
    /// 
    ///     Execution will iterate through every robot pose in the trajectory pose 
    ///     array while updating the joint values on the simulated robot.
    /// 
    /// </summary>
    /// <param name="trajectories"> The array of poses for the robot to execute</param>
    private IEnumerator ExecuteTrajectories(RobotTrajectoryMsg trajectories)
    {
        // For every robot pose in trajectory plan
        foreach (var point in trajectories.joint_trajectory.points)
        {
            var jointPositions = point.positions;
            float[] result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

            // Set the joint values for every joint
            for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = jointArticulationBodies[joint].xDrive;
                joint1XDrive.target = result[joint];
                jointArticulationBodies[joint].xDrive = joint1XDrive;
            }
            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(JOINT_ASSIGNMENT_WAIT);
        }
    }
}
