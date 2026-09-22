#if UNITY_SIMULATION_FOUNDATION
using UnityEngine;
using Unity.Simulation.Foundation;
using RosMessageTypes.VehicleControllers;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter will capture input from ROS and convert it into a message
    /// that then gets passed on to the controller. 
    /// </summary>
    public class MastConnectorAdapter : MonoBehaviour, IMastAdapter
    {
        [SerializeField]
        private string m_Topic = "mast";
        private IConnector ros;
        private IRobotController<MastControlMessage> m_Controller;

        void Start()
        {
            ros = ConnectorInjector.FindConnector(this);
            if (ros != null)
                ros.Subscribe<MastMsg>(m_Topic, ReceiveMastMsg);

            m_Controller = this.FetchControllerWithMessageType<MastControlMessage>();
        }

        void ReceiveMastMsg(MastMsg msg)
        {
            var message = new MastControlMessage()
            {
                mastVerticalExtension = msg.mast_vertical_extension,
                mastVerticalSpeed = msg.mast_vertical_speed,
                forkLongitudinalExtension = msg.fork_longitudinal_extension,
                forkLongitudinalSpeed = msg.fork_longitudinal_speed,
                forkTiltTarget = msg.fork_tilt_target,
                forkTiltSpeed = msg.fork_tilt_speed,
                forkLateralExtension = msg.fork_lateral_extension,
                forkLateralSpeed = msg.fork_lateral_speed,
            };
            m_Controller.ConsumeMessage(message);
        }
    }
}
#endif
