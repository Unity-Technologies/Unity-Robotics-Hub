using System;
using UnityEngine;
using UnityEngine.Assertions;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This script controls the ArticulationBody components for the Differential Drive
    /// type vehicle. It accepts an DifferentialDriveControlMessage and converts that into
    /// ArticulationBody drive values. 
    /// </summary>
    public class DifferentialDriveController : MonoBehaviour, IRobotController<DifferentialDriveControlMessage>
    {
        /// <summary>
        /// A structure that holds the left and right ArticulationBody wheels along
        /// with some information about them - their radius and their forward facing direction.
        /// </summary>
        [Serializable]
        public struct WheelPair
        {
            public ArticulationBody leftWheel;
            public ArticulationBody rightWheel;
            public float wheelRadius;
            // Public so that odometry can undo the same axis normalisation the controller applies when
            // commanding. Both are derived in DrivingWheelSetup(); treat them as read-only.
            public float leftWheelSign;
            public float rightWheelSign;
        }

        [Tooltip("The driving wheels of the vehicle")]
        [SerializeField] public WheelPair[] m_WheelPairs;

        private float m_TrackWidth;

        private DifferentialDriveControlMessage m_LastMessage;

        /// <summary>
        /// All the pairs of wheels that are assigned to this controller.
        /// </summary>
        public WheelPair[] WheelPairs { get { return m_WheelPairs; } set { m_WheelPairs = value; } }

        /// <summary>
        /// Last message received by this controller.
        /// </summary>
        public DifferentialDriveControlMessage LastMessage => m_LastMessage;

        /// <summary>
        /// The distance between the left and right wheels averaged for all the pairs. Measured in meters.
        /// </summary>
        public float TrackWidth { get { return m_TrackWidth; } set { m_TrackWidth = value; } }

        /// <summary>
        /// Stores <paramref name="message"/> as the pending command without touching the wheels.
        /// Call <see cref="SetVelocity"/> to apply it, or use <see cref="ConsumeMessage"/> to do both.
        /// </summary>
        /// <remarks>
        /// The message is interpreted in Unity's left-handed convention: forward speed is
        /// <c>linear.z</c> (m/s) and yaw rate is <c>angular.y</c> (rad/s, positive turns right when
        /// viewed from above). A ROS <c>geometry_msgs/Twist</c> must be converted before it gets here -
        /// see <c>DifferentialDriveConnectorAdapter</c>.
        /// </remarks>
        public void SetCommand(DifferentialDriveControlMessage message)
        {
            m_LastMessage = message;
        }

        /// <summary>
        /// Applies the last command stored by <see cref="SetCommand"/> to the wheel drives.
        /// </summary>
        public void SetVelocity()
        {
            if (m_WheelPairs == null)
                return;

            float leftWheelRotationSpeed;
            float rightWheelRotationSpeed;
            float addedAngularRotationalSpeed;

            foreach (var wheelPair in m_WheelPairs)
            {
                leftWheelRotationSpeed = m_LastMessage.linear.z / wheelPair.wheelRadius;
                rightWheelRotationSpeed = leftWheelRotationSpeed;

                addedAngularRotationalSpeed = m_LastMessage.angular.y * m_TrackWidth / wheelPair.wheelRadius;

                leftWheelRotationSpeed = (leftWheelRotationSpeed + addedAngularRotationalSpeed / 2f) * Mathf.Rad2Deg;
                rightWheelRotationSpeed = (rightWheelRotationSpeed - addedAngularRotationalSpeed / 2f) * Mathf.Rad2Deg;

                wheelPair.leftWheel.SetXDriveTargetVelocity(leftWheelRotationSpeed * wheelPair.leftWheelSign);
                wheelPair.rightWheel.SetXDriveTargetVelocity(rightWheelRotationSpeed * wheelPair.rightWheelSign);
            }
        }

        // Start is called before the first frame update
        private void Start()
        {
            for (int i = 0; i < m_WheelPairs.Length; i++)
                DrivingWheelSetup(ref m_WheelPairs[i]);

            ComputeTrackWidth();
        }

        private void DrivingWheelSetup(ref WheelPair wheelPair)
        {
            Debug.Assert(wheelPair.leftWheel, "Left wheel is not set!");
            Debug.Assert(wheelPair.rightWheel, "Right wheel is not set!");

            var leftCollider = wheelPair.leftWheel.GetComponentInChildren<Collider>();
            var rightCollider = wheelPair.rightWheel.GetComponentInChildren<Collider>();

            if (leftCollider == null)
            {
                Debug.LogError($"Left wheel {wheelPair.leftWheel.name} ArticulationBody has no collider!");
                return;
            }

            if (rightCollider == null)
            {
                Debug.LogError($"Right wheel {wheelPair.rightWheel.name} ArticulationBody has no collider!");
                return;
            }

            //We're using bounds.extents.y instead of radius, because radius is not affected by scale
            if (rightCollider.bounds.extents.y != leftCollider.bounds.extents.y)
                Debug.LogError($"Left wheel radius and Right wheel radius are not identical! L: {leftCollider.bounds.extents.y}, R: {rightCollider.bounds.extents.y}");

            Assert.AreNotEqual(0f, leftCollider.bounds.extents.y, $"Left wheel {wheelPair.leftWheel.name} radius was 0f!");
            Assert.AreNotEqual(0f, rightCollider.bounds.extents.y, $"Right wheel {wheelPair.rightWheel.name} radius was 0f!");

            wheelPair.wheelRadius = leftCollider.bounds.extents.y;

            Quaternion leftWheelAnchorRotation;
            Quaternion rightWheelAnchorRotation;

            if (Application.isPlaying)
            {
                leftWheelAnchorRotation = wheelPair.leftWheel.anchorRotation;
                rightWheelAnchorRotation = wheelPair.rightWheel.anchorRotation;
            }
            else
            {
                leftWheelAnchorRotation = wheelPair.leftWheel.parentAnchorRotation;
                rightWheelAnchorRotation = wheelPair.rightWheel.parentAnchorRotation;
            }

            Vector3 leftAxisDirection = wheelPair.leftWheel.transform.rotation * leftWheelAnchorRotation * Vector3.right;
            Vector3 rightAxisDirection = wheelPair.rightWheel.transform.rotation * rightWheelAnchorRotation * Vector3.right;

            var sidewaysDirection = wheelPair.rightWheel.transform.position - wheelPair.leftWheel.transform.position;

            wheelPair.leftWheelSign = Mathf.Sign(Vector3.Dot(sidewaysDirection.normalized, leftAxisDirection));
            wheelPair.rightWheelSign = Mathf.Sign(Vector3.Dot(sidewaysDirection.normalized, rightAxisDirection));
        }

        private void ComputeTrackWidth()
        {
            var leftSideMidPoint = Vector3.zero;
            var rightSideMidPoint = Vector3.zero;

            if (m_WheelPairs.Length > 0)
            {
                foreach (var wheelPair in m_WheelPairs)
                {
                    leftSideMidPoint += wheelPair.leftWheel.transform.position;
                    rightSideMidPoint += wheelPair.rightWheel.transform.position;
                }

                m_TrackWidth = Vector3.Distance(leftSideMidPoint / m_WheelPairs.Length, rightSideMidPoint / m_WheelPairs.Length);
            }
        }


        /// <inheritdoc />
        public void ConsumeMessage(DifferentialDriveControlMessage message)
        {
            SetCommand(message);
            SetVelocity();
        }
    }
}
