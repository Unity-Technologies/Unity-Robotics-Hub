using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter will capture input from the keyboard and convert it into a message
    /// that then gets passed on to the controller. 
    /// </summary>
    public class ThreeWheelKeyboardAdapter : MonoBehaviour, IThreeWheelAdapter
    {
        [SerializeField]
        private float m_MaxSteeringAngle = 45f; // Deg
        [SerializeField]
        private float m_SteeringVelocity = 45f; // Deg/s
        [SerializeField]
        private float m_MaxLinearVelocity = 2f; // m/s
        [SerializeField]
        private float m_LinearAcceleration = 2f;// m/s^2
        [SerializeField]
        private float m_LinearJerk = 0f;        // m/s^3

        [SerializeField]
        private InputActionAsset m_ThreeWheelInputAsset;

        private IRobotController<ThreeWheelControlMessage> m_Controller;
        private InputActionMap m_InputActionMap;
        private InputAction m_DriveAction;
        private InputAction m_BrakeAction;

        private void Start()
        {
            m_Controller = this.FetchControllerWithMessageType<ThreeWheelControlMessage>();

            if (m_ThreeWheelInputAsset == null)
            {
                Debug.LogError("Three Wheel Drive Keyboard Adapter Input Asset not set. Input will not be captured.", m_ThreeWheelInputAsset);
                return;
            }

            m_InputActionMap = m_ThreeWheelInputAsset.FindActionMap("MainControl");

            m_DriveAction = m_InputActionMap.FindAction("Drive");
            m_BrakeAction = m_InputActionMap.FindAction("Brake");

            m_DriveAction.Enable();
            m_BrakeAction.Enable();
        }

        private void Update()
        {
            if (m_ThreeWheelInputAsset == null)
                return;

            var drive = m_DriveAction.ReadValue<Vector2>(); // X: A/D  Y: W/S
            var isBraking = m_BrakeAction.ReadValue<bool>(); // Space

            var message = new ThreeWheelControlMessage()
            {
                steeringAngle = (drive.x * -1) * m_MaxSteeringAngle * Mathf.Deg2Rad,
                speed = isBraking ? 0f : m_MaxLinearVelocity * drive.y,
                acceleration = isBraking ? 0f : m_LinearAcceleration,
                steeringAngleVelocity = m_SteeringVelocity * Mathf.Deg2Rad,
                jerk = m_LinearJerk
            };

            m_Controller.ConsumeMessage(message);
        }
    }
}
