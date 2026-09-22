using System;
using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.GeometryMessages;
using Unity.Simulation.VehicleControllers;

namespace SimProIntegration
{
    /// <summary>
    /// Bridges ROS <c>geometry_msgs/Twist</c> messages on a topic (default "/cmd_vel") into
    /// <see cref="DifferentialDriveControlMessage"/> commands on a <see cref="DifferentialDriveController"/>.
    ///
    /// Keeping this separate from the controller means the drive itself has no ROS dependency, and the
    /// robot can be driven from a test harness or keyboard script by calling
    /// <see cref="DifferentialDriveController.ConsumeMessage"/> directly.
    ///
    /// Incoming Twists are in ROS/FLU (forward = linear.x, yaw rate = angular.z, positive turns left).
    /// The controller expects Unity's left-handed convention (forward = linear.z, yaw rate = angular.y,
    /// positive turns right). This bridge is where that conversion happens - see <c>Update</c>.
    ///
    /// Only one adapter may drive the controller: ConsumeMessage applies immediately, so a second
    /// adapter calling it each frame (e.g. DifferentialDriveKeyboardAdapter, which sends zeros when no
    /// key is held) will fight this one non-deterministically. Disable the others.
    ///
    /// This script is NOT part of the SimulationPro package (com.unity.simulationpro is read-only) -
    /// copy it into your own Unity project's Assets/Scripts folder.
    /// </summary>
    public class CmdVelDifferentialDriveBridge : SubscriberBehaviour<TwistMsg>
    {
        [Header("Safety")]
        [Tooltip("If no /cmd_vel message arrives within this many seconds, the robot is commanded to stop. " +
                 "Prevents a lost ROS connection from leaving the robot driving forever.")]
        public float CommandTimeout = 0.5f;

        [Header("Limits (optional)")]
        [Tooltip("Clamp applied to incoming linear.x, in m/s. Set to 0 to disable clamping.")]
        public float MaxLinearSpeed = 2.0f;
        [Tooltip("Clamp applied to incoming angular.z, in rad/s. Set to 0 to disable clamping.")]
        public float MaxAngularSpeed = 3.0f;

        DifferentialDriveController m_Controller;

        // RosEndpointConnector drains its incoming queue from MainThreadLoop, so these callbacks arrive on
        // the Unity main thread and touching UnityEngine APIs here is safe. Commands are still buffered and
        // applied in Update so that several messages arriving within one frame collapse to the latest one.
        float m_LinearX;
        float m_AngularZ;
        float m_LastCommandTime = float.NegativeInfinity;

        /// <inheritdoc />
        public override string DefaultTopic => "/cmd_vel";

        // Cached so the property doesn't allocate a fresh delegate on every received message -
        // SubscriberBehaviour reads Callback inside its per-message dispatch lambda.
        Action<TwistMsg> m_Callback;

        /// <inheritdoc />
        protected override Action<TwistMsg> Callback => m_Callback ??= OnTwistReceived;

        protected override void Start()
        {
            m_Controller = GetComponent<DifferentialDriveController>();
            if (m_Controller == null)
                Debug.LogError($"{name}'s {nameof(CmdVelDifferentialDriveBridge)} found no " +
                    $"{nameof(DifferentialDriveController)} on this GameObject; {Topic} will be ignored.", this);

            base.Start();
        }

        void OnTwistReceived(TwistMsg msg)
        {
            float linear = (float)msg.linear.x;
            float angular = (float)msg.angular.z;

            if (MaxLinearSpeed > 0f)
                linear = Mathf.Clamp(linear, -MaxLinearSpeed, MaxLinearSpeed);
            if (MaxAngularSpeed > 0f)
                angular = Mathf.Clamp(angular, -MaxAngularSpeed, MaxAngularSpeed);

            m_LinearX = linear;
            m_AngularZ = angular;
            m_LastCommandTime = Time.time;
        }

        void Update()
        {
            if (m_Controller == null)
                return;

            if (Time.time - m_LastCommandTime > CommandTimeout)
            {
                // An all-zero message is a full stop in either convention.
                m_Controller.ConsumeMessage(default);
                return;
            }

            // DifferentialDriveControlMessage is consumed in Unity's LEFT-HANDED convention - the
            // controller reads linear.z and angular.y and ignores every other component. Writing the
            // ROS/FLU fields (linear.x, angular.z) here means the controller reads zeros and the robot
            // never moves, which is silent and very easy to miss.
            m_Controller.ConsumeMessage(new DifferentialDriveControlMessage
            {
                // ROS x-forward becomes Unity z-forward.
                linear = new Vector3(0f, 0f, m_LinearX),
                // ROS +z yaw is counter-clockwise seen from above; Unity +y yaw is clockwise.
                angular = new Vector3(0f, -m_AngularZ, 0f)
            });
        }
    }
}
