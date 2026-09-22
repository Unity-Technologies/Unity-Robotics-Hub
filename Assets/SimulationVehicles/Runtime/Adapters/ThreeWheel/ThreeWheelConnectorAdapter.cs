#if UNITY_SIMULATION_FOUNDATION
using System.Linq;
using Unity.Simulation.Foundation;
using UnityEngine;
using RosMessageTypes.VehicleControllers;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter will capture input from ROS as an Ackermann message and
    /// convert it into a Unity message that then gets passed on to the controller. 
    /// </summary>
    public class ThreeWheelConnectorAdapter : MonoBehaviour, IThreeWheelAdapter
    {
        [SerializeField]
        private string m_Topic = "ackermann_drive";
        private IConnector con;
        private IRobotController<ThreeWheelControlMessage> m_Controller;

        private void Start()
        {
            con = ConnectorInjector.FindConnector(this);
            if (con != null)
                con.Subscribe<AckermannDriveMsg>(m_Topic, ReceiveThreeWheelMsg);

            m_Controller = this.FetchControllerWithMessageType<ThreeWheelControlMessage>();
        }

        void ReceiveThreeWheelMsg(AckermannDriveMsg msg)
        {
            var message = new ThreeWheelControlMessage()
            {
                steeringAngle = msg.steering_angle,
                speed = msg.speed,
                acceleration = msg.acceleration,
                steeringAngleVelocity = msg.steering_angle_velocity,
                jerk = msg.jerk
            };

            m_Controller.ConsumeMessage(message);
        }

    }
}
#endif
