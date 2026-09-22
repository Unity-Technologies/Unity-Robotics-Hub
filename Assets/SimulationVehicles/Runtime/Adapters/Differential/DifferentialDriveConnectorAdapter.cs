#if UNITY_SIMULATIONPRO_FOUNDATION
using UnityEngine;
using Unity.SimulationPro.Foundation;
using Unity.SimulationPro.Foundation.GeometryMessages;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// Subscribes to a ROS <c>geometry_msgs/Twist</c> topic and converts each message into a
    /// <see cref="DifferentialDriveControlMessage"/> for the <see cref="DifferentialDriveController"/>
    /// on this GameObject.
    /// </summary>
    /// <remarks>
    /// ROS bodies use REP-103 / FLU: <c>linear.x</c> is forward in m/s and <c>angular.z</c> is yaw rate
    /// in rad/s, positive turning left. The controller consumes Unity's left-handed convention, where
    /// forward is <c>linear.z</c> and yaw is <c>angular.y</c> with the opposite sense. Both the axis
    /// remap and the yaw sign flip happen here in <see cref="ReceiveTwistMsg"/>, which keeps the
    /// controller itself free of any ROS dependency.
    /// </remarks>
    public class DifferentialDriveConnectorAdapter : MonoBehaviour, IDifferentialDriveAdapter
    {
        [SerializeField]
        [Tooltip("Twist topic to subscribe to.")]
        private string m_Topic = "/cmd_vel";

        [SerializeField]
        [Tooltip("Clamp applied to the incoming forward speed, in m/s. Set to 0 to pass the commanded value through unclamped.")]
        private float m_MaxLinearSpeed = 1.25f;

        [SerializeField]
        [Tooltip("Clamp applied to the incoming yaw rate, in rad/s. Set to 0 to pass the commanded value through unclamped.")]
        private float m_MaxAngularSpeed = 3f;

        private IConnector m_Connector;
        private IRobotController<DifferentialDriveControlMessage> m_Controller;

        void Start()
        {
            m_Controller = this.FetchControllerWithMessageType<DifferentialDriveControlMessage>();

            m_Connector = ConnectorInjector.FindConnector(this);
            if (m_Connector == null)
            {
                Debug.LogWarning($"{name}'s {nameof(DifferentialDriveConnectorAdapter)} couldn't find a valid " +
                    $"connector, can't subscribe to {m_Topic}", this);
                return;
            }

            m_Connector.Subscribe<TwistMsg>(m_Topic, ReceiveTwistMsg);
        }

        void ReceiveTwistMsg(TwistMsg msg)
        {
            // IConnector exposes no Unsubscribe, so the connector keeps dispatching to this callback
            // even after the component is destroyed. Bail rather than throw on a dead reference.
            if (this == null || m_Controller == null)
                return;

            var forward = (float)msg.linear.x;
            var yawRate = (float)msg.angular.z;

            if (m_MaxLinearSpeed > 0f)
                forward = Mathf.Clamp(forward, -m_MaxLinearSpeed, m_MaxLinearSpeed);
            if (m_MaxAngularSpeed > 0f)
                yawRate = Mathf.Clamp(yawRate, -m_MaxAngularSpeed, m_MaxAngularSpeed);

            m_Controller.ConsumeMessage(new DifferentialDriveControlMessage
            {
                // ROS x-forward becomes Unity z-forward.
                linear = new Vector3(0f, 0f, forward),
                // ROS +z yaw is counter-clockwise seen from above; Unity +y yaw is clockwise.
                angular = new Vector3(0f, -yawRate, 0f)
            });
        }
    }
}
#endif
