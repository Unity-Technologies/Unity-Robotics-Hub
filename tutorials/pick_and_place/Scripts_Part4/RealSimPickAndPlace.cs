using System;
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
    const int k_OpenGripper = 1;
    const int k_CloseGripper = 2;
    const int k_ToolCommandExecution = 6;
    const int k_TrajectoryCommandExecution = 7;
    const int k_NumRobotJoints = 6;

    // Hardcoded variables
    const float k_JointAssignmentWait = 0.038f;

    // Variables required for ROS communication
    public string rosJointPublishTopicName = "sim_real_pnp";
    public string rosRobotCommandsTopicName = "niryo_one/commander/robot_action/goal";

    [SerializeField]
    GameObject m_NiryoOne;
    [SerializeField]
    GameObject m_Target;
    [SerializeField]
    GameObject m_TargetPlacement;

    // Assures that the gripper is always positioned above the m_Target cube before grasping.
    readonly Quaternion m_PickOrientation = Quaternion.Euler(90, 90, 0);
    readonly Vector3 m_PickPoseOffset = Vector3.up * 0.15f;

    // Articulation Bodies
    ArticulationBody[] m_JointArticulationBodies;

    ArticulationBody m_LeftGripper;
    ArticulationBody m_RightGripper;

    ROSConnection m_Ros;

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find left and right finger joints and assign them to their respective articulation body objects.
    /// </summary>
    void Awake()
    {
        m_JointArticulationBodies = new ArticulationBody[k_NumRobotJoints];
        var linkName = "world/base_link/shoulder_link";
        m_JointArticulationBodies[0] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();

        linkName += "/arm_link";
        m_JointArticulationBodies[1] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();

        linkName += "/elbow_link";
        m_JointArticulationBodies[2] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();

        linkName += "/forearm_link";
        m_JointArticulationBodies[3] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();

        linkName += "/wrist_link";
        m_JointArticulationBodies[4] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();

        linkName += "/hand_link";
        m_JointArticulationBodies[5] = m_NiryoOne.transform.Find(linkName).GetComponent<ArticulationBody>();

        // Find left and right fingers
        var rightGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_right/right_gripper";
        var leftGripper = linkName + "/tool_link/gripper_base/servo_head/control_rod_left/left_gripper";

        m_RightGripper = m_NiryoOne.transform.Find(rightGripper).GetComponent<ArticulationBody>();
        m_LeftGripper = m_NiryoOne.transform.Find(leftGripper).GetComponent<ArticulationBody>();
    }

    void Start()
    {
        // Get ROS connection static instance
        m_Ros = ROSConnection.GetOrCreateInstance();
        m_Ros.Subscribe<RobotMoveActionGoal>(rosRobotCommandsTopicName, ExecuteRobotCommands);
    }

    /// <summary>
    ///     Close the gripper
    /// </summary>
    void CloseGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = 0.01f;
        rightDrive.target = -0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Open the gripper
    /// </summary>
    void OpenGripper()
    {
        var leftDrive = m_LeftGripper.xDrive;
        var rightDrive = m_RightGripper.xDrive;

        leftDrive.target = -0.01f;
        rightDrive.target = 0.01f;

        m_LeftGripper.xDrive = leftDrive;
        m_RightGripper.xDrive = rightDrive;
    }

    /// <summary>
    ///     Publish the robot's current joint configuration, the m_Target object's
    ///     position and rotation, and the m_Target placement for the m_Target object's
    ///     position and rotation.
    ///     Includes conversion from Unity coordinates to ROS coordinates, Forward Left Up
    /// </summary>
    public void PublishJoints()
    {
        var request = new MoverServiceRequest
        {
            joints_input = new NiryoMoveitJointsMsg(),
            pick_pose = new PoseMsg
            {
                position = (m_Target.transform.position + m_PickPoseOffset).To<FLU>(),

                // The hardcoded x/z angles assure that the gripper is always positioned above the m_Target cube before grasping.
                orientation = Quaternion.Euler(90, m_Target.transform.eulerAngles.y, 0).To<FLU>()
            },
            place_pose = new PoseMsg
            {
                position = (m_TargetPlacement.transform.position + m_PickPoseOffset).To<FLU>(),
                orientation = m_PickOrientation.To<FLU>()
            }
        };

        for (var i = 0; i < k_NumRobotJoints; i++)
            request.joints_input.joints[i] = m_JointArticulationBodies[i].jointPosition[0];

        m_Ros.Send(rosJointPublishTopicName, request);
    }

    /// <summary>
    ///     Execute robot commands receved from ROS Subscriber.
    ///     Gripper commands will be executed immeditately wihle trajectories will be
    ///     executed in a coroutine.
    /// </summary>
    /// <param name="robotAction"> RobotMoveActionGoal of trajectory or gripper commands</param>
    void ExecuteRobotCommands(RobotMoveActionGoal robotAction)
    {
        if (robotAction.goal.cmd.cmd_type == k_TrajectoryCommandExecution)
        {
            StartCoroutine(ExecuteTrajectories(robotAction.goal.cmd.Trajectory.trajectory));
        }
        else if (robotAction.goal.cmd.cmd_type == k_ToolCommandExecution)
        {
            if (robotAction.goal.cmd.tool_cmd.cmd_type == k_OpenGripper)
            {
                Debug.Log("Open Tool Command");
                OpenGripper();
            }
            else if (robotAction.goal.cmd.tool_cmd.cmd_type == k_CloseGripper)
            {
                Debug.Log("Close Tool Command");
                CloseGripper();
            }
        }
    }

    /// <summary>
    ///     Execute trajectories from RobotMoveActionGoal topic.
    ///     Execution will iterate through every robot pose in the trajectory pose
    ///     array while updating the joint values on the simulated robot.
    /// </summary>
    /// <param name="trajectories"> The array of poses for the robot to execute</param>
    IEnumerator ExecuteTrajectories(RobotTrajectoryMsg trajectories)
    {
        // For every robot pose in trajectory plan
        foreach (var point in trajectories.joint_trajectory.points)
        {
            var jointPositions = point.positions;
            var result = jointPositions.Select(r => (float)r * Mathf.Rad2Deg).ToArray();

            // Set the joint values for every joint
            for (var joint = 0; joint < m_JointArticulationBodies.Length; joint++)
            {
                var joint1XDrive = m_JointArticulationBodies[joint].xDrive;
                joint1XDrive.target = result[joint];
                m_JointArticulationBodies[joint].xDrive = joint1XDrive;
            }

            // Wait for robot to achieve pose for all joint assignments
            yield return new WaitForSeconds(k_JointAssignmentWait);
        }
    }
}
