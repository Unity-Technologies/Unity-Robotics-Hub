using System;
using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.GeometryMessages;

namespace SimProIntegration
{
    /// <summary>
    /// Drives a Rigidbody-based robot chassis from geometry_msgs/Twist commands received on a ROS 2 topic
    /// (default "/cmd_vel"). Assumes a differential-drive-style robot using the ROS REP-103 body-frame
    /// convention: <c>linear.x</c> is forward speed in m/s, <c>angular.z</c> is yaw rate in rad/s where a
    /// positive value turns the robot left (counter-clockwise viewed from above).
    ///
    /// Attach this to the robot's root GameObject alongside a Rigidbody. Motion is applied kinematically via
    /// Rigidbody.MovePosition/MoveRotation in FixedUpdate, so it works with a simple non-physically-driven
    /// chassis (a real wheeled robot could instead map linear/angular into per-wheel speeds, but this is the
    /// simplest way to get a first working Unity <-> ROS 2 loop running).
    ///
    /// This script is NOT part of the SimulationPro package (com.unity.simulationpro is read-only) -
    /// copy it into your own Unity project's Assets/Scripts folder.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class CmdVelDriveController : SubscriberBehaviour<TwistMsg>
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

        Rigidbody m_Rigidbody;
        float m_LinearX;
        float m_AngularZ;
        float m_LastCommandTime = float.NegativeInfinity;

        /// <inheritdoc />
        public override string DefaultTopic => "/cmd_vel";

        /// <inheritdoc />
        protected override Action<TwistMsg> Callback => OnTwistReceived;

        protected override void Start()
        {
            m_Rigidbody = GetComponent<Rigidbody>();
            m_Rigidbody.isKinematic = true; // moved manually from ROS commands, not by physics forces
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

        void FixedUpdate()
        {
            float linear = m_LinearX;
            float angular = m_AngularZ;

            if (Time.time - m_LastCommandTime > CommandTimeout)
            {
                linear = 0f;
                angular = 0f;
            }

            // ROS (FLU): +angular.z turns the robot left (CCW viewed from above +Z).
            // Unity (RUF, left-handed): a positive rotation about +Y turns "forward" toward "right",
            // i.e. the opposite sense - so we negate angular.z here to match ROS's convention.
            Vector3 forward = m_Rigidbody.rotation * Vector3.forward;
            Vector3 nextPosition = m_Rigidbody.position + forward * (linear * Time.fixedDeltaTime);
            Quaternion deltaRotation = Quaternion.Euler(0f, -angular * Mathf.Rad2Deg * Time.fixedDeltaTime, 0f);
            Quaternion nextRotation = m_Rigidbody.rotation * deltaRotation;

            m_Rigidbody.MovePosition(nextPosition);
            m_Rigidbody.MoveRotation(nextRotation);
        }
    }
}
