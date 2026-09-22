using System;
using System.Collections;
using UnityEngine;

#if UNITY_SIMULATION_FOUNDATION
using Unity.Simulation.Foundation;
using Unity.Simulation.Foundation.NavMessages;
using Unity.Simulation.Foundation.StdMessages;
using Unity.Simulation.Foundation.BuiltinInterfacesMessages;
using Unity.Simulation.Foundation.GeometryMessages;
#endif

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// The odometry publisher takes the wheel position difference in the robot
    /// and converts that into a pose and twist estimate. Alongside it provides
    /// the pose and twist covariance matrices.
    /// </summary>
    [RequireComponent(typeof(DifferentialDriveController))]
    public class DifferentialDriveOdometryPublisher : MonoBehaviour
    {
        private DifferentialDriveController.WheelPair[] m_WheelPairs;

        [SerializeField]
        private ArticulationBody m_BaseLink;

#if UNITY_SIMULATION_FOUNDATION
        [SerializeField]
        private string m_OdomTopic = "/odometry";
        private string m_BaseLinkID = "UnityAMR_Prefab";
        private IConnector m_Ros;
        private IPublisher m_Pub;

        const double k_NanosecondsInSecond = 1e9f;
        double m_LastPublishTimeSeconds;

        [SerializeField]
        float m_PublishRateHz = 20f;
        double m_PublishPeriodSeconds => 1.0f / m_PublishRateHz;
        bool m_TimeToPublishMessage => Clock.Now > m_LastPublishTimeSeconds + m_PublishPeriodSeconds;
        [SerializeField]
        bool m_SendMessagesToRos = true;
        bool m_ConnectionValid = false;
#endif

        [SerializeField]
        private bool m_DrawGizmos;

        [SerializeField]
        private float m_PredictedHeading = 0;
        [SerializeField]
        private Vector3 m_PredictedPosition;
        [SerializeField]
        private Vector3 m_PredictedVelocity;

        private struct WheelPairPos
        {
            public float leftWheelPos;
            public float rightWheelPos;
        }

        private WheelPairPos[] m_CurWheelPairPos;
        private WheelPairPos[] m_PrevWheelPairPos;

        private float m_VelZ, m_VelX, m_VelHeading;
        private float m_WheelSeparation;
        private float m_DeltaHeading = 0;

        private Vector3 m_ActualPosition;
        private Vector3 m_ActualVelocity;
        private Vector3 m_PreviousForwardDir;
        private float m_ActualHeading;
        private Vector3 m_ActualAngularVelocity;

        private DifferentialDriveController m_Controller;

        [HideInInspector]
        private double[] m_PoseCovariance = new double[36];
        [HideInInspector]
        private double[] m_TwistCovariance = new double[36];

        /// <summary>
        /// The pose covariance related to the odometry data that is being generated.
        /// The covariance is defined by measuring the difference of the odometry
        /// position output against the ground truth in simulation.
        /// </summary>
        public double[] PoseCovariance { get { return m_PoseCovariance; } set { m_PoseCovariance = value; } }

        /// <summary>
        /// The twist covariance related to the odometry data that is being generated.
        /// The covariance is defined by measuring the difference of the odometry
        /// velocity output against the ground truth in simulation.
        /// </summary>
        public double[] TwistCovariance { get { return m_TwistCovariance; } set { m_TwistCovariance = value; } }

        bool b_Initialized = false;

        void Start()
        {
            b_Initialized = false;

            m_Controller = GetComponent<DifferentialDriveController>();
            m_WheelPairs = m_Controller.WheelPairs;
            m_CurWheelPairPos = new WheelPairPos[m_WheelPairs.Length];
            m_PrevWheelPairPos = new WheelPairPos[m_WheelPairs.Length];
            m_PreviousForwardDir = m_BaseLink.transform.forward;

            StartCoroutine(LateStart());
#if UNITY_SIMULATION_FOUNDATION
            m_BaseLinkID = m_BaseLink.name;

            m_Ros = ConnectorInjector.FindConnector(this);

            if (m_Ros != null)
            {
                m_ConnectionValid = true;
                m_Pub = m_Ros.RegisterPublisher<OdometryMsg>(m_OdomTopic);
            }
            else
            {
                m_ConnectionValid = false;
                Debug.LogWarning("Connection to Ros was not established.");
            }
            m_LastPublishTimeSeconds = Clock.Now + m_PublishPeriodSeconds;
#else
            Debug.LogWarning("Simulation Foundation package not found. Publishing to ROS will not work.");
#endif
            GetCurWheelPos();
            SetPrevWheelPos();
        }

        IEnumerator LateStart()
        {
            yield return null;
            m_WheelSeparation = m_Controller.TrackWidth;

            b_Initialized = true;
        }

        void FixedUpdate()
        {
            if (!b_Initialized)
                return;

            GetActualValues();
            IntegratePoseAndTwist();
            UpdateCovariance();
        }

        void GetActualValues()
        {
            m_ActualPosition = m_BaseLink.transform.position;
            m_ActualVelocity = m_BaseLink.linearVelocity;
            // SignedAngle clamps the angle to (-180:180) range, but since we only use it to get the delta value,
            // we should never run into this problem unless the robot turns around 180 degrees in a single frame
            m_ActualHeading += Vector3.SignedAngle(m_PreviousForwardDir, m_BaseLink.transform.forward, Vector3.up);
            m_ActualAngularVelocity = m_BaseLink.angularVelocity;

            m_PreviousForwardDir = m_BaseLink.transform.forward;
        }

        void IntegratePoseAndTwist()
        {
            GetCurWheelPos();
            float deltaLeft = 0;
            float deltaRight = 0;
            for (int i = 0; i < m_WheelPairs.Length; i++)
            {
                deltaLeft += (m_CurWheelPairPos[i].leftWheelPos - m_PrevWheelPairPos[i].leftWheelPos) * m_WheelPairs[i].wheelRadius * m_WheelPairs[i].leftWheelSign;
                deltaRight += (m_CurWheelPairPos[i].rightWheelPos - m_PrevWheelPairPos[i].rightWheelPos) * m_WheelPairs[i].wheelRadius * m_WheelPairs[i].rightWheelSign;
            }

            deltaLeft /= m_WheelPairs.Length;
            deltaRight /= m_WheelPairs.Length;

            m_DeltaHeading = (deltaLeft - deltaRight) / m_WheelSeparation;
            var deltaCenterPos = (deltaLeft + deltaRight) / 2;

            m_PredictedHeading += m_DeltaHeading;
            var deltaZ = deltaCenterPos * Mathf.Cos(m_PredictedHeading);
            var deltaX = deltaCenterPos * Mathf.Sin(m_PredictedHeading);

            m_VelHeading = m_DeltaHeading / Time.fixedDeltaTime;
            m_VelZ = deltaZ / Time.fixedDeltaTime;
            m_VelX = deltaX / Time.fixedDeltaTime;

            m_PredictedPosition += new Vector3(deltaX, 0, deltaZ);
            m_PredictedVelocity = new Vector3(m_VelX, 0, m_VelZ);

            SetPrevWheelPos();
        }

        void UpdateCovariance()
        {
            // POSE. Unity coords

            // cov(X, X)
            m_PoseCovariance[0] = m_PredictedPosition.x - m_ActualPosition.x;
            m_PoseCovariance[0] *= m_PoseCovariance[0];

            //cov(Z, Z)
            m_PoseCovariance[14] = m_PredictedPosition.z - m_ActualPosition.z;
            m_PoseCovariance[14] *= m_PoseCovariance[14];

            // cov(Pitch, Pitch)
            m_PoseCovariance[28] = m_PredictedHeading - m_ActualHeading * Mathf.Deg2Rad;
            m_PoseCovariance[28] *= m_PoseCovariance[28];

            // TWIST. Unity coords

            // cov(Vx,Vx)
            m_TwistCovariance[0] = m_VelX - m_ActualVelocity.x;
            m_TwistCovariance[0] *= m_TwistCovariance[0];

            // cov(Vz,Vz)
            m_TwistCovariance[14] = m_VelZ - m_ActualVelocity.z;
            m_TwistCovariance[14] *= m_TwistCovariance[14];

            // cov (ωy, ωy)
            m_TwistCovariance[28] = m_VelHeading - m_ActualAngularVelocity.y * Mathf.Deg2Rad;
            m_TwistCovariance[28] *= m_TwistCovariance[28];

        }

        void SetPrevWheelPos()
        {
            for (int i = 0; i < m_WheelPairs.Length; i++)
            {
                m_PrevWheelPairPos[i].leftWheelPos = m_WheelPairs[i].leftWheel.jointPosition[0];
                m_PrevWheelPairPos[i].rightWheelPos = m_WheelPairs[i].rightWheel.jointPosition[0];
            }
        }

        void GetCurWheelPos()
        {
            for (int i = 0; i < m_WheelPairs.Length; i++)
            {
                m_CurWheelPairPos[i].leftWheelPos = m_WheelPairs[i].leftWheel.jointPosition[0];
                m_CurWheelPairPos[i].rightWheelPos = m_WheelPairs[i].rightWheel.jointPosition[0];
            }
        }


        private void OnDrawGizmos()
        {
            if (m_DrawGizmos)
            {
                Matrix4x4 originalSpace = Gizmos.matrix;
                Matrix4x4 predictedSpace = Matrix4x4.TRS(m_PredictedPosition, Quaternion.Euler(new Vector3(0, m_PredictedHeading * Mathf.Rad2Deg, 0)), Vector3.one);

                Gizmos.matrix = predictedSpace;

                Gizmos.color = Color.red;
                Gizmos.DrawCube(Vector3.zero, new Vector3(0.5f, 0.2f, 0.5f));

                Gizmos.color = Color.white;
                Gizmos.DrawCube(new Vector3(0, 0, 0.4f), new Vector3(0.15f, 0.15f, 0.2f));

                Gizmos.matrix = originalSpace;
            }
        }

#if UNITY_SIMULATION_FOUNDATION
        void Update()
        {
            if (m_TimeToPublishMessage && m_SendMessagesToRos && m_ConnectionValid)
            {
                PublishMessage();
            }
        }

        // Update Odom
        void UpdateMessage(OdometryMsg odomMessage)
        {
            odomMessage.header = new HeaderMsg();
            odomMessage.header.stamp = new TimeMsg();
            odomMessage.header.stamp.sec = (int)Math.Floor(Clock.Now);
            odomMessage.header.stamp.nanosec = (uint)((Clock.Now - odomMessage.header.stamp.sec) * k_NanosecondsInSecond);

            odomMessage.header.frame_id = "odom";// "map";
            odomMessage.child_frame_id = m_BaseLinkID;

            Vector3<FLU> predictedPosROS = new Vector3<FLU>(m_PredictedPosition);
            Quaternion<FLU> predictedRotROS = new Quaternion<FLU>(Quaternion.Euler(new Vector3(0, m_PredictedHeading * Mathf.Rad2Deg, 0)));

            odomMessage.pose = new PoseWithCovarianceMsg();
            odomMessage.pose.pose = new PoseMsg();
            odomMessage.pose.covariance = m_PoseCovariance;
            odomMessage.pose.pose.position.x = predictedPosROS.x;
            odomMessage.pose.pose.position.y = predictedPosROS.y;
            odomMessage.pose.pose.position.z = predictedPosROS.z;
            odomMessage.pose.pose.orientation.x = predictedRotROS.x;
            odomMessage.pose.pose.orientation.y = predictedRotROS.y;
            odomMessage.pose.pose.orientation.z = predictedRotROS.z;
            odomMessage.pose.pose.orientation.w = predictedRotROS.w;

            Vector3<FLU> predictedVelROS = new Vector3<FLU>(m_PredictedVelocity);

            odomMessage.twist = new TwistWithCovarianceMsg();
            odomMessage.twist.twist = new TwistMsg();
            odomMessage.twist.covariance = m_TwistCovariance;
            odomMessage.twist.twist.linear.x = predictedVelROS.x;
            odomMessage.twist.twist.linear.y = predictedVelROS.y;
            odomMessage.twist.twist.linear.z = 0f;
            odomMessage.twist.twist.angular.x = 0f;
            odomMessage.twist.twist.angular.y = 0f;
            odomMessage.twist.twist.angular.z = m_VelHeading;
        }

        void PublishMessage()
        {
            var odomMessage = new OdometryMsg();
            UpdateMessage(odomMessage);

            m_Pub.Publish(odomMessage);
            m_LastPublishTimeSeconds = Clock.Now;
        }
#endif
    }
}
