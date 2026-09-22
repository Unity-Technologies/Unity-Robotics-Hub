#if UNITY_SIMULATION_FOUNDATION
using System.Linq;
using Unity.Simulation.Foundation;
using UnityEngine;
using RosMessageTypes.VehicleControllers;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter will capture input from ROS and convert it into a message
    /// that then gets passed on to the controller. 
    /// </summary>
    public class AckermannConnectorAdapter : MonoBehaviour, IAckermannAdapter
    {
        [SerializeField]
        private string m_Topic = "ackermann_drive";

        private IRobotController<AckermannControlMessage> m_Controller;

        private void Start()
        {
        var con = ConnectorInjector.FindConnector(this);
            if (con != null)
                con.Subscribe<AckermannDriveMsg>(m_Topic, ReceiveAckermannMsg);

            m_Controller = this.FetchControllerWithMessageType<AckermannControlMessage>();
        }

        void ReceiveAckermannMsg(AckermannDriveMsg msg)
        {
            var message = new AckermannControlMessage()
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
