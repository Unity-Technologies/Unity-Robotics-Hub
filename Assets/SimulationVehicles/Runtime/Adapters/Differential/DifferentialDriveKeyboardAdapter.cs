using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter will capture input from the keyboard and convert it into a message
    /// that then gets passed on to the controller. 
    /// </summary>
    public class DifferentialDriveKeyboardAdapter : MonoBehaviour, IDifferentialDriveAdapter
    {
        [SerializeField] private float m_MaxLinearSpeed = 0.2f;         // (m/s)
        [SerializeField] private float m_MaxAngularSpeed = 45f;           // (degrees/s)

        [SerializeField] private InputActionAsset m_DifferentialInput;

        private IRobotController<DifferentialDriveControlMessage> m_Controller;
        private InputActionMap m_InputActionMap;
        private InputAction m_DriveAction;

        private void Start()
        {
            m_Controller = this.FetchControllerWithMessageType<DifferentialDriveControlMessage>();

            if (m_DifferentialInput == null)
            {
                Debug.LogError("Differential Drive Keyboard Adapter Input Asset not set. Input will not be captured.", m_DifferentialInput);
                return;
            }

            m_InputActionMap = m_DifferentialInput.FindActionMap("MainControl");

            m_DriveAction = m_InputActionMap.FindAction("Drive");

            m_DriveAction.Enable();
        }

        private void Update()
        {
            if (m_DifferentialInput == null)
                return;

            var drive = m_DriveAction.ReadValue<Vector2>(); // X: A/D  Y: W/S

            var message = new DifferentialDriveControlMessage()
            {
                linear = new Vector3(0, 0, drive.y * m_MaxLinearSpeed),
                angular = new Vector3(0, drive.x * m_MaxAngularSpeed * Mathf.Deg2Rad, 0)
            };
            m_Controller.ConsumeMessage(message);
        }
    }
}
