using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;
using UnityEngine.UI;

using ROSGeometry;
using RosMessageTypes.Ur3Moveit;
using Quaternion = UnityEngine.Quaternion;
using Transform = UnityEngine.Transform;
using Vector3 = UnityEngine.Vector3;

public class TrajectoryPlanner : MonoBehaviour
{
    // ROS Connector
    private ROSConnection ros;

    // Hardcoded variables 
    private int numRobotJoints = 6;
    private readonly float jointAssignmentWait = 0.06f;
    private readonly float poseAssignmentWait = 0.5f;
    private readonly float gripperAngle = 14f;
    // Offsets to ensure gripper is above grasp points
    private readonly Vector3 pickPoseOffset = new Vector3(0, 0.255f, 0);
    private readonly Vector3 placePoseOffset = new Vector3(0, 0.275f, 0);
    // Multipliers correspond to the URDF mimic tag for each joint
    private float[] multipliers = new float[] { 1f, 1f, -1f, -1f, 1f, -1f };
    // Orientation is hardcoded for this example so the gripper is always directly above the placement object
    private readonly Quaternion pickOrientation = new Quaternion(-0.5f,-0.5f,0.5f,-0.5f);

    // Variables required for ROS communication
    public string rosServiceName = "ur3_moveit";
    private const int isBigEndian = 0;
    private const int step = 4;

    public GameObject robot;
    public GameObject target;
    public Transform goal;

    // Articulation Bodies
    private ArticulationBody[] jointArticulationBodies;
    ArticulationBody[] articulationChain;
    private List<ArticulationBody> gripperJoints;

    // UI elements
    private Button InitializeButton;
    private Button RandomizeButton;
    private Button ServiceButton;
    private Text ActualPos;
    private Text ActualRot;
    private Text EstimatedPos;
    private Text EstimatedRot;

    private RenderTexture renderTexture;

    public GameObject scenario;

    private enum Poses
    {
        PreGrasp,
        Grasp,
        PickUp,
        Place,
        PostPlace
    };

    /// <summary>
    ///     Opens and closes the attached gripper tool based on a gripping angle.
    /// </summary>
    /// <param name="toClose"></param>
    /// <returns></returns>
    public IEnumerator IterateToGrip(bool toClose)
    {
        var grippingAngle = toClose ? gripperAngle : 0f;
        for (int i = 0; i < gripperJoints.Count; i++)
        {
            var curXDrive = gripperJoints[i].xDrive;
            curXDrive.target = multipliers[i] * grippingAngle;
            gripperJoints[i].xDrive = curXDrive;
        }
        yield return new WaitForSeconds(jointAssignmentWait);
    }

    /// <summary>
    ///     Button callback for setting the robot to default position
    /// </summary>
    public void Initialize(){
        StartCoroutine(MoveToInitialPosition());
    }

    /// <summary>
    ///     Button callback for the Cube Randomization
    /// </summary>
    public void RandomizeCube(){
        InferenceRandomization.Move(scenario);
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
    }

    /// <summary>
    ///     Button callback for the Pose Estimation
    /// </summary>
    public void PoseEstimation(){
        Debug.Log("Capturing screenshot...");

        InitializeButton.interactable = false;
        RandomizeButton.interactable = false;
        ServiceButton.interactable = false;
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
        EstimatedPos.text = "-";
        EstimatedRot.text = "-";

        // Capture the screenshot and pass it to the pose estimation service
        byte[] rawImageData = CaptureScreenshot();
        InvokePoseEstimationService(rawImageData);
    }

    private IEnumerator MoveToInitialPosition()
    {
        bool isRotationFinished = false;
        while (!isRotationFinished)
        {
            isRotationFinished = ResetRobotToDefaultPosition();
            yield return new WaitForSeconds(jointAssignmentWait);
        }
        ServiceButton.interactable = true;
    }

    private bool ResetRobotToDefaultPosition()
    {
        bool isRotationFinished = true;
        var rotationSpeed = 180f;

        for (int i = 1; i < numRobotJoints + 1; i++)
        {
            var tempXDrive = articulationChain[i].xDrive;
            float currentRotation = tempXDrive.target;
            
            float rotationChange = rotationSpeed * Time.fixedDeltaTime;
            
            if (currentRotation > 0f) rotationChange *= -1;
            
            if (Mathf.Abs(currentRotation) < rotationChange)
                rotationChange = 0;
            else
                isRotationFinished = false;
            
            // the new xDrive target is the currentRotation summed with the desired change
            float rotationGoal = currentRotation + rotationChange;
            tempXDrive.target = rotationGoal;
            articulationChain[i].xDrive = tempXDrive;
        }
        return isRotationFinished;
    }

    /// <summary>
    ///     Create a new PoseEstimationServiceRequest with the captured screenshot as bytes and instantiates 
    ///     a sensor_msgs/image.
    ///
    ///     Call the PoseEstimationService using the ROSConnection and calls PoseEstimationCallback on the 
    ///     PoseEstimationServiceResponse.
    /// </summary>
    /// <param name="imageData"></param>
    private void InvokePoseEstimationService(byte[] imageData)
    {
        uint imageHeight = (uint)renderTexture.height;
        uint imageWidth = (uint)renderTexture.width;

        RosMessageTypes.Sensor.Image rosImage = new RosMessageTypes.Sensor.Image(new RosMessageTypes.Std.Header(), imageWidth, imageHeight, "RGBA", isBigEndian, step, imageData);
        PoseEstimationServiceRequest poseServiceRequest = new PoseEstimationServiceRequest(rosImage);
        ros.SendServiceMessage<PoseEstimationServiceResponse>("pose_estimation_srv", poseServiceRequest, PoseEstimationCallback);
	}

    void PoseEstimationCallback(PoseEstimationServiceResponse response)
	{
        if (response != null)
        {
            // The position output by the model is the position of the cube relative to the camera so we need to extract its global position 
            var estimatedPosition = Camera.main.transform.TransformPoint(response.estimated_pose.position.From<RUF>());
            var estimatedRotation = Camera.main.transform.rotation * response.estimated_pose.orientation.From<RUF>();

            PublishJoints(estimatedPosition, estimatedRotation);

            EstimatedPos.text = estimatedPosition.ToString();
            EstimatedRot.text = estimatedRotation.eulerAngles.ToString();
        }
        else {
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
        }
    }

    /// <summary>
    ///     Capture the main camera's render texture and convert to bytes.
    /// </summary>
    /// <returns>imageBytes</returns>
    private byte[] CaptureScreenshot()
    {
        Camera.main.targetTexture = renderTexture;
        RenderTexture currentRT = RenderTexture.active;
        RenderTexture.active = renderTexture;
        Camera.main.Render();
        Texture2D mainCameraTexture = new Texture2D(renderTexture.width, renderTexture.height);
        mainCameraTexture.ReadPixels(new Rect(0, 0, renderTexture.width, renderTexture.height), 0, 0);
        mainCameraTexture.Apply();
        RenderTexture.active = currentRT;
        // Get the raw byte info from the screenshot
        byte[] imageBytes = mainCameraTexture.GetRawTextureData();
        Camera.main.targetTexture = null;
        return imageBytes;
    }

    /// <summary>
    ///     Get the current values of the robot's joint angles.
    /// </summary>
    /// <returns>UR3MoveitJoints</returns>
    UR3MoveitJoints CurrentJointConfig()
    {
        UR3MoveitJoints joints = new UR3MoveitJoints();
        
        joints.joint_00 = jointArticulationBodies[0].xDrive.target;
        joints.joint_01 = jointArticulationBodies[1].xDrive.target;
        joints.joint_02 = jointArticulationBodies[2].xDrive.target;
        joints.joint_03 = jointArticulationBodies[3].xDrive.target;
        joints.joint_04 = jointArticulationBodies[4].xDrive.target;
        joints.joint_05 = jointArticulationBodies[5].xDrive.target;

        return joints;
    }

    public void PublishJoints(Vector3 targetPos, Quaternion targetRot)
    {
        MoverServiceRequest request = new MoverServiceRequest();
        request.joints_input = CurrentJointConfig();

        // Pick Pose
        request.pick_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (targetPos + pickPoseOffset).To<FLU>(),
            orientation = Quaternion.Euler(90, targetRot.eulerAngles.y, 0).To<FLU>()
        };

        // Place Pose
        request.place_pose = new RosMessageTypes.Geometry.Pose
        {
            position = (goal.position + placePoseOffset).To<FLU>(),
            orientation = pickOrientation.To<FLU>()
        };

        ros.SendServiceMessage<MoverServiceResponse>(rosServiceName, request, TrajectoryResponse);
    }

    void TrajectoryResponse(MoverServiceResponse response)
    {
        if (response.trajectories != null && response.trajectories.Length > 0)
        {
            Debug.Log("Trajectory returned.");
            StartCoroutine(ExecuteTrajectories(response));
        }
        else
        {
            Debug.LogError("No trajectory returned from MoverService.");
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
            ServiceButton.interactable = true;
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
    /// <param name="response"> MoverServiceResponse received from ur3_moveit mover service running in ROS</param>
    /// <returns></returns>
    private IEnumerator ExecuteTrajectories(MoverServiceResponse response)
    {
        if (response.trajectories != null)
        {
            // For every trajectory plan returned
            for (int poseIndex  = 0 ; poseIndex < response.trajectories.Length; poseIndex++)
            {
                // For every robot pose in trajectory plan
                for (int jointConfigIndex  = 0 ; jointConfigIndex < response.trajectories[poseIndex].joint_trajectory.points.Length; jointConfigIndex++)
                {
                    var jointPositions = response.trajectories[poseIndex].joint_trajectory.points[jointConfigIndex].positions;
                    float[] result = jointPositions.Select(r=> (float)r * Mathf.Rad2Deg).ToArray();
                    
                    // Set the joint values for every joint
                    for (int joint = 0; joint < jointArticulationBodies.Length; joint++)
                    {
                        var joint1XDrive  = jointArticulationBodies[joint].xDrive;
                        joint1XDrive.target = result[joint];
                        jointArticulationBodies[joint].xDrive = joint1XDrive;
                    }
                    // Wait for robot to achieve pose for all joint assignments
                    yield return new WaitForSeconds(jointAssignmentWait);
                }

                // Close the gripper if completed executing the trajectory for the Grasp pose
                if (poseIndex == (int)Poses.Grasp){
                    StartCoroutine(IterateToGrip(true));
                    yield return new WaitForSeconds(jointAssignmentWait);
                }
                else if (poseIndex == (int)Poses.Place){
                    yield return new WaitForSeconds(poseAssignmentWait);
                    // Open the gripper to place the target cube
                    StartCoroutine(IterateToGrip(false));
                }
                // Wait for the robot to achieve the final pose from joint assignment
                yield return new WaitForSeconds(poseAssignmentWait);
            }

            // Re-enable buttons
            InitializeButton.interactable = true;
            RandomizeButton.interactable = true;
            yield return new WaitForSeconds(jointAssignmentWait);
        }
    }

    /// <summary>
    ///     Find all robot joints in Awake() and add them to the jointArticulationBodies array.
    ///     Find all gripper joints and assign them to their respective articulation body objects.
    /// </summary>
    void Awake()
    {
        jointArticulationBodies = new ArticulationBody[numRobotJoints];
        string shoulder_link = "world/base_link/shoulder_link";
        jointArticulationBodies[0] = robot.transform.Find(shoulder_link).GetComponent<ArticulationBody>();

        string arm_link = shoulder_link + "/upper_arm_link";
        jointArticulationBodies[1] = robot.transform.Find(arm_link).GetComponent<ArticulationBody>();
        
        string elbow_link = arm_link + "/forearm_link";
        jointArticulationBodies[2] = robot.transform.Find(elbow_link).GetComponent<ArticulationBody>();
        
        string forearm_link = elbow_link + "/wrist_1_link";
        jointArticulationBodies[3] = robot.transform.Find(forearm_link).GetComponent<ArticulationBody>();
        
        string wrist_link = forearm_link + "/wrist_2_link";
        jointArticulationBodies[4] = robot.transform.Find(wrist_link).GetComponent<ArticulationBody>();
        
        string hand_link = wrist_link + "/wrist_3_link";
        jointArticulationBodies[5] = robot.transform.Find(hand_link).GetComponent<ArticulationBody>();

        articulationChain = robot.GetComponent<RosSharp.Control.Controller>().GetComponentsInChildren<ArticulationBody>();

        var gripperJointNames = new string[] { "right_outer_knuckle", "right_inner_finger", "right_inner_knuckle", "left_outer_knuckle", "left_inner_finger", "left_inner_knuckle" };
        gripperJoints = new List<ArticulationBody>();

        foreach (ArticulationBody articulationBody in robot.GetComponentsInChildren<ArticulationBody>())
        {
            if (gripperJointNames.Contains(articulationBody.name))
            {
                gripperJoints.Add(articulationBody);
            }
        }
    }

    void Start()
    {
        // Get ROS connection static instance
        ros = ROSConnection.instance;
        
        // Assign UI elements
        InitializeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/DefaultButton").GetComponent<Button>();
        RandomizeButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/RandomButton").GetComponent<Button>();
        ServiceButton = GameObject.Find("ROSObjects/Canvas/ButtonPanel/ServiceButton").GetComponent<Button>();
        
        ActualPos = GameObject.Find("ROSObjects/Canvas/PositionPanel/ActualPosField").GetComponent<Text>();
        ActualRot = GameObject.Find("ROSObjects/Canvas/PositionPanel/ActualRotField").GetComponent<Text>();
        EstimatedPos = GameObject.Find("ROSObjects/Canvas/PositionPanel/EstPosField").GetComponent<Text>();
        EstimatedRot = GameObject.Find("ROSObjects/Canvas/PositionPanel/EstRotField").GetComponent<Text>();

        // Initialize UI element values
        ActualPos.text = target.transform.position.ToString();
        ActualRot.text = target.transform.eulerAngles.ToString();
        EstimatedPos.text = "-";
        EstimatedRot.text = "-";

        // Render texture 
        renderTexture = new RenderTexture(Camera.main.pixelWidth, Camera.main.pixelHeight, 24, UnityEngine.Experimental.Rendering.GraphicsFormat.R8G8B8A8_SRGB);
        renderTexture.Create();
    }
}