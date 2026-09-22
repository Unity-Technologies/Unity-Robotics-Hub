using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.StdMessages;
using Unity.SimulationPro.Foundation.GeometryMessages;
using Unity.SimulationPro.Foundation.NavMessages;

namespace SimProIntegration
{
    /// <summary>
    /// Publishes nav_msgs/Odometry for this robot's Rigidbody on a ROS 2 topic (default "/odom").
    /// This is what a Python navigation node reads to know the robot's current position/heading -
    /// without it, a waypoint-following node has no way to tell how close it is to the goal.
    ///
    /// Requires a Rigidbody on the same GameObject. Pose comes from Rigidbody.position/rotation and
    /// velocity from Rigidbody.linearVelocity/angularVelocity.
    ///
    /// Set "Publish Frequency" in the Inspector (AutoPublisher default is a very low 0.01 Hz) - something
    /// in the 10-30 Hz range is reasonable for odometry.
    ///
    /// This script is NOT part of the SimulationPro package (com.unity.simulationpro is read-only) -
    /// copy it into your own Unity project's Assets/Scripts folder.
    /// </summary>
    [RequireComponent(typeof(Rigidbody))]
    public class OdometryPublisher : AutoPublisher<OdometryMsg>
    {
        [Header("Frames")]
        [Tooltip("Fixed/world frame this odometry is reported in (header.frame_id).")]
        public string OdomFrameId = "odom";
        [Tooltip("Frame attached to the robot body (child_frame_id).")]
        public string ChildFrameId = "base_link";

        Rigidbody m_Rigidbody;

        /// <inheritdoc />
        public override string DefaultTopic => "/odom";

        protected override void Start()
        {
            m_Rigidbody = GetComponent<Rigidbody>();
            base.Start();
        }

        protected override OdometryMsg CreateMessage()
        {
            // Note: if the robot is driven by a kinematic Rigidbody (as CmdVelDriveController does via
            // MovePosition), these velocity reads return zero - pose stays correct, but twist will be
            // empty. See the README for the two ways to get real velocities into /odom.
            var positionFLU = new Vector3<FLU>(m_Rigidbody.position);
            var rotationFLU = new Quaternion<FLU>(m_Rigidbody.rotation);
            var linearVelocityFLU = m_Rigidbody.linearVelocity.To<FLU>();
            var angularVelocityFLU = Vector3<FLU>.FromUnityAngularVelocity(m_Rigidbody.angularVelocity);

            var header = new HeaderMsg(ClockHelpers.Now(), OdomFrameId);
            var pose = new PoseMsg(positionFLU, rotationFLU);
            var twist = new TwistMsg(linearVelocityFLU, angularVelocityFLU);

            // Placeholder covariance - replace the diagonal with real sensor/estimator uncertainty if
            // this feeds anything (e.g. an EKF or Nav2 costmap) that actually consumes it.
            var poseCovariance = new double[36];
            var twistCovariance = new double[36];
            for (int i = 0; i < 6; i++)
            {
                poseCovariance[i * 6 + i] = 1e-3;
                twistCovariance[i * 6 + i] = 1e-3;
            }

            return new OdometryMsg(
                header,
                ChildFrameId,
                new PoseWithCovarianceMsg(pose, poseCovariance),
                new TwistWithCovarianceMsg(twist, twistCovariance));
        }
    }
}
