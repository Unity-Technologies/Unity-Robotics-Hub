#if UNITY_SIMULATION_FOUNDATION
using System.Linq;
using System;
using Unity.Simulation.Foundation;
using UnityEngine;
using Unity.Simulation.Foundation.GeometryMessages;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter will capture input from ROS as a twist message and
    /// convert it into a Unity message that then gets passed on to the controller.
    /// </summary>
    public class ThreeWheelConnectorTwistAdapter : MonoBehaviour, IThreeWheelAdapter
    {
        [SerializeField]
        private string m_Topic = "cmd_vel";

        private IConnector m_Con;
        private ThreeWheelController m_Controller;

        private void Start()
        {
            m_Con = ConnectorInjector.FindConnector(this);
            if (m_Con != null)
                m_Con.Subscribe<TwistMsg>(m_Topic, ReceiveTwistMsg);

            m_Controller = (ThreeWheelController)this.FetchControllerWithMessageType<ThreeWheelControlMessage>();
        }

        private void ReceiveTwistMsg(TwistMsg msg)
        {
            var vx = msg.linear.x;
            var vy = m_Controller.Wheelbase * msg.angular.z;
            var angle = -Math.Atan2(vy,vx);
            var message = new ThreeWheelControlMessage()
            {
                steeringAngle = (float)angle,
                speed = (float)Math.Sqrt(vx*vx + vy*vy),
                acceleration = 0,
                steeringAngleVelocity = 0,
                jerk = 0
            };

            m_Controller.ConsumeMessage(message);
        }
    }
}
#endif
