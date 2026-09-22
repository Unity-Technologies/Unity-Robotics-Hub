using System.Collections.Generic;
using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.StdMessages;
using Unity.SimulationPro.Foundation.GeometryMessages;
using Unity.SimulationPro.Foundation.NavMessages;
using Unity.Simulation.VehicleControllers;
using System.Linq;

namespace SimProIntegration
{
    /// <summary>
    /// Publishes <c>nav_msgs/Odometry</c> dead-reckoned from actual wheel rotation - i.e. real wheel
    /// encoder odometry, not ground truth.
    ///
    /// Wheel speeds are read from the ArticulationBody joints the
    /// <see cref="DifferentialDriveController"/> drives, then run through differential-drive forward
    /// kinematics and integrated over time. Because it measures what the wheels actually did rather than
    /// where the robot actually is, it drifts on wheel slip exactly like a physical robot - which is the
    /// point: it lets you test whether navigation still works under imperfect localization.
    ///
    /// The pose is integrated directly in the ROS/FLU convention, so the "odom" frame origin is the
    /// robot's pose at startup, with +x along the robot's initial forward and +y to its initial left.
    /// No Unity-to-ROS axis conversion is involved, so waypoint coordinates are simply meters forward
    /// and left of where the robot started.
    ///
    /// Set "Publish Frequency" in the Inspector (AutoPublisher default is a very low 0.01 Hz) - something
    /// in the 10-30 Hz range is reasonable for odometry.
    ///
    /// This script is NOT part of the SimulationPro package (com.unity.simulationpro is read-only) -
    /// copy it into your own Unity project's Assets/Scripts folder.
    /// </summary>
    
    public class WheelOdometryPublisher : AutoPublisher<OdometryMsg>
    {
        [Header("Frames")]
        [Tooltip("Fixed/world frame this odometry is reported in (header.frame_id).")]
        public string OdomFrameId = "odom";
        [Tooltip("Frame attached to the robot body (child_frame_id).")]
        public string ChildFrameId = "base_link";

        DifferentialDriveController m_Drive;

        // Dead-reckoned pose in the odom frame (ROS convention: x forward, y left, yaw CCW).
        double m_X;
        double m_Y;
        double m_Yaw;

        // Latest measured body velocities, reported in the twist field.
        double m_MeasuredLinear;
        double m_MeasuredAngular;

        // Integrate() runs every frame, so the missing-radius error is logged once rather than forever.
        bool m_WarnedNoWheelRadius;

        /// <inheritdoc />
        public override string DefaultTopic => "/odom";

        /// <summary>Dead-reckoned X position in the odom frame, in meters.</summary>
        public double EstimatedX => m_X;

        /// <summary>Dead-reckoned Y position in the odom frame, in meters.</summary>
        public double EstimatedY => m_Y;

        /// <summary>Dead-reckoned heading in the odom frame, in radians.</summary>
        public double EstimatedYaw => m_Yaw;


        // Runtime read-outs, all mirrored from DifferentialDriveController - useful for confirming at a
        // glance that the odometry is wired to the wheels you expect. Editing them in the Inspector has
        // no effect: the integration reads the controller's wheel pairs directly. wheelRadius is the one
        // exception, acting as a fallback if the controller reports 0.
        [Header("Runtime read-outs (from the controller)")]
        public List<ArticulationBody> leftWheels;
        public List<ArticulationBody> rightWheels;
        public float wheelRadius;
        public float wheelSeparation;

        protected override void Start()
        {
            m_Drive = GetComponent<DifferentialDriveController>();
            if (m_Drive == null)
            {
                Debug.LogError($"{name}'s {nameof(WheelOdometryPublisher)} found no " +
                    $"{nameof(DifferentialDriveController)} on this GameObject; {Topic} will report a " +
                    $"stationary robot.", this);
                base.Start();
                return;
            }

            // WheelPair.leftWheel/rightWheel are serialized, so they are populated by the time any
            // Start() runs. WheelPair.wheelRadius is NOT - see ResolveWheelRadius for why it can't be
            // cached here.
            leftWheels ??= new List<ArticulationBody>();
            rightWheels ??= new List<ArticulationBody>();
            leftWheels.Clear();
            rightWheels.Clear();

            foreach (var pair in m_Drive.WheelPairs)
            {
                leftWheels.Add(pair.leftWheel);
                rightWheels.Add(pair.rightWheel);
            }

            base.Start();
        }

        /// <summary>
        /// Returns the drive wheel radius in meters, preferring the controller's measured value.
        /// </summary>
        /// <remarks>
        /// DifferentialDriveController derives WheelPair.wheelRadius from the wheel collider bounds in
        /// its own Start(), and Unity guarantees no ordering between two MonoBehaviours on the same
        /// GameObject. Caching the radius in our Start() is therefore a coin flip: lose the race and we
        /// store 0, every surface speed becomes 0, and /odom reports a permanently stationary robot for
        /// the rest of the session - which reads as a navigation bug rather than an odometry one.
        /// Resolving per-integration instead costs an array access and self-corrects as soon as the
        /// controller has initialised, exactly as the TrackWidth read below already does. The Inspector
        /// field is kept in sync so it doubles as a live read-out, and is used as the fallback if the
        /// controller has no usable value.
        /// </remarks>
        float ResolveWheelRadius()
        {
            if (m_Drive != null && m_Drive.WheelPairs != null && m_Drive.WheelPairs.Length > 0)
            {
                float measured = m_Drive.WheelPairs[0].wheelRadius;
                if (measured > 0f)
                    wheelRadius = measured;
            }

            return wheelRadius;
        }

        /// <summary>
        /// Integrates the odometry estimate, then lets the base class handle publish timing.
        /// </summary>
        /// <remarks>
        /// Integration lives here rather than in FixedUpdate because AutoPublisher's FixedUpdate is not
        /// virtual and hiding it would break the base class's publish scheduling. Integrating per-frame
        /// with Time.deltaTime is accurate enough for odometry at typical frame rates; if you need
        /// strictly physics-stepped integration, move the Integrate() call into a small companion
        /// component's FixedUpdate.
        /// </remarks>
        protected override void Update()
        {
            // Clock.deltaTime rather than Time.deltaTime, so the estimate stays consistent if the project
            // switches Clock away from UnityUnscaled (e.g. FoundationClock or ExternalClock).
            Integrate(Clock.deltaTime);

            // PublisherBehaviour.Publish dereferences Publisher unguarded, so publishing with no connector
            // in the scene would throw every tick. Keep integrating regardless - dead reckoning shouldn't
            // develop a gap just because ROS isn't attached yet.
            if (!CanPublish)
                return;

            base.Update();
        }

        void Integrate(float deltaTime)
        {
            if (deltaTime <= 0f || m_Drive == null)
                return;

            float radius = ResolveWheelRadius();
            if (radius <= 0f)
            {
                if (!m_WarnedNoWheelRadius)
                {
                    m_WarnedNoWheelRadius = true;
                    Debug.LogError($"{name}'s {nameof(WheelOdometryPublisher)} has no usable wheel radius: " +
                        $"the controller reported 0 and the Inspector value is {wheelRadius}. Odometry will " +
                        $"report a stationary robot until this is set.", this);
                }
                return;
            }

            ReadWheelSpeeds(out float leftWheelSpeed, out float rightWheelSpeed);

            // Differential drive forward kinematics: wheel spin -> body velocity.
            double leftSurfaceSpeed = leftWheelSpeed * radius;    // m/s
            double rightSurfaceSpeed = rightWheelSpeed * radius;  // m/s

            // TrackWidth is the controller's own measurement, taken between the wheel transforms, so
            // commanded and measured kinematics stay consistent. Mirrored into the Inspector field,
            // which is otherwise unused and would be misleading to tune.
            double separation = Mathf.Max(m_Drive.TrackWidth, 1e-4f);
            wheelSeparation = (float)separation;
            m_MeasuredLinear = 0.5 * (leftSurfaceSpeed + rightSurfaceSpeed);
            m_MeasuredAngular = (rightSurfaceSpeed - leftSurfaceSpeed) / separation;

            // Midpoint integration - noticeably less heading error on curves than plain Euler.
            double midYaw = m_Yaw + 0.5 * m_MeasuredAngular * deltaTime;
            m_X += m_MeasuredLinear * System.Math.Cos(midYaw) * deltaTime;
            m_Y += m_MeasuredLinear * System.Math.Sin(midYaw) * deltaTime;
            m_Yaw = NormalizeAngle(m_Yaw + m_MeasuredAngular * deltaTime);
        }

        /// <summary>
        /// Averages measured wheel speed per side, in rad/s, positive meaning the robot's forward
        /// direction.
        /// </summary>
        /// <remarks>
        /// Read straight off the controller's wheel pairs rather than the mirrored lists, because the
        /// per-wheel sign lives there and is only valid after the controller's Start() has run - see
        /// ResolveWheelRadius for the same ordering argument. Integrate() is called from Update(), by
        /// which point every Start() has completed, so the signs are settled.
        /// </remarks>
        void ReadWheelSpeeds(out float left, out float right)
        {
            left = 0f;
            right = 0f;

            if (m_Drive == null || m_Drive.WheelPairs == null)
                return;

            float leftTotal = 0f, rightTotal = 0f;
            int leftCount = 0, rightCount = 0;

            foreach (var pair in m_Drive.WheelPairs)
            {
                if (TryReadWheelSpeed(pair.leftWheel, pair.leftWheelSign, out float l))
                {
                    leftTotal += l;
                    leftCount++;
                }

                if (TryReadWheelSpeed(pair.rightWheel, pair.rightWheelSign, out float r))
                {
                    rightTotal += r;
                    rightCount++;
                }
            }

            if (leftCount > 0)
                left = leftTotal / leftCount;
            if (rightCount > 0)
                right = rightTotal / rightCount;
        }

        static bool TryReadWheelSpeed(ArticulationBody wheel, float sign, out float speed)
        {
            speed = 0f;

            if (wheel == null || wheel.dofCount == 0)
                return false; // a fixed joint has no DOF, so jointVelocity[0] would be out of range

            // jointVelocity is in RADIANS per second (unlike xDrive targets, which are in degrees).
            // Reading velocity rather than differencing jointPosition avoids having to handle the angle
            // wrapping of a continuously spinning wheel.
            //
            // The sign undoes the joint-axis normalisation that DifferentialDriveController applies when
            // it commands the same wheel. Without it, a robot whose wheel axes oppose the left->right
            // vector - which is every URDF that gives both wheels axis (0,1,0), including UnityAMR -
            // integrates its pose backwards while driving forwards.
            speed = wheel.jointVelocity[0] * sign;
            return true;
        }

        static double NormalizeAngle(double angle)
        {
            while (angle > System.Math.PI)
                angle -= 2.0 * System.Math.PI;
            while (angle <= -System.Math.PI)
                angle += 2.0 * System.Math.PI;
            return angle;
        }

        /// <summary>
        /// Resets the dead-reckoned pose to the origin. Useful when re-homing the robot mid-run, since
        /// accumulated odometry drift never corrects itself on its own.
        /// </summary>
        public void ResetOdometry()
        {
            m_X = 0.0;
            m_Y = 0.0;
            m_Yaw = 0.0;
        }

        protected override OdometryMsg CreateMessage()
        {
            // Pose is already in ROS convention, so it is built directly rather than going through
            // Vector3<FLU>/Quaternion<FLU>.
            var position = new PointMsg(m_X, m_Y, 0.0);
            var orientation = new QuaternionMsg(
                0.0,
                0.0,
                System.Math.Sin(m_Yaw * 0.5),
                System.Math.Cos(m_Yaw * 0.5));

            var header = new HeaderMsg(ClockHelpers.Now(), OdomFrameId);
            var pose = new PoseMsg(position, orientation);

            // Per REP-103 the twist is expressed in child_frame_id (the robot body), so forward speed
            // goes in linear.x and yaw rate in angular.z with no rotation applied.
            var twist = new TwistMsg(
                new Vector3Msg(m_MeasuredLinear, 0.0, 0.0),
                new Vector3Msg(0.0, 0.0, m_MeasuredAngular));

            // Placeholder covariance. Wheel odometry uncertainty genuinely grows without bound as drift
            // accumulates, so if you later feed an EKF, replace these with values that reflect that.
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
