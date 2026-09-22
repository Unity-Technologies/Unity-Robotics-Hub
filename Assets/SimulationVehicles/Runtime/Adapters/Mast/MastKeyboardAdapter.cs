using System;
using System.Linq;
using UnityEngine;
using UnityEngine.InputSystem;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter will capture input from the keyboard and convert it into a message
    /// that then gets passed on to the controller. 
    /// </summary>
    public class MastKeyboardAdapter : MonoBehaviour, IMastAdapter
    {
        [SerializeField]
        private float m_MastVerticalTarget = 3.9f;          // m
        [SerializeField]
        private float m_MastVerticalSpeed = 1f;             // m/s
        [SerializeField]
        private float m_ForkLongitudinalTarget = 1f;          // m
        [SerializeField]
        private float m_ForkLongitudinalSpeed = 1f;           // m/s
        [SerializeField]
        private float m_ForkTiltTarget = 5f;                // Deg
        [SerializeField]
        private float m_ForkTiltSpeed = 4f;                 // Deg/s
        [SerializeField]
        private float m_ForkLateralAdjustmentTarget = 0.15f;// m
        [SerializeField]
        private float m_ForkLateralAdjustmentSpeed = 1f;    // m/s

        [SerializeField]
        private InputActionAsset m_MastInputAsset;

        private IRobotController<MastControlMessage> m_Controller;
        private InputActionMap m_InputActionMap;
        private InputAction m_MastVerticalAction;
        private InputAction m_ForkLongitudinalAction;
        private InputAction m_ForkLateralAction;
        private InputAction m_ForkTiltAction;

        private float mastVertical = 0;
        private float forkLongitudinal = 0;
        private float forkLateral = 0;
        private float forkTilt = 0;
        private void Start()
        {
            m_Controller = this.FetchControllerWithMessageType<MastControlMessage>();

            if (m_MastInputAsset == null)
            {
                Debug.LogError("Mast Keyboard Adapter Input Asset not set. Input will not be captured.", m_MastInputAsset);
                return;
            }

            m_InputActionMap = m_MastInputAsset.FindActionMap("MainControl");
            m_MastVerticalAction = m_InputActionMap.FindAction("MastVertical");
            m_ForkLongitudinalAction = m_InputActionMap.FindAction("ForkLongitudinal");
            m_ForkLateralAction = m_InputActionMap.FindAction("ForkLateral");
            m_ForkTiltAction = m_InputActionMap.FindAction("ForkTilt");

            m_MastVerticalAction.Enable();
            m_ForkLongitudinalAction.Enable();
            m_ForkLateralAction.Enable();
            m_ForkTiltAction.Enable();
        }

        private void Update()
        {
            if (m_MastInputAsset == null)
                return;

            mastVertical = GetInput(mastVertical, m_MastVerticalAction, 0, m_MastVerticalTarget, m_MastVerticalSpeed); // Up arrow to go up, down arrow to go down.
            forkLongitudinal = GetInput(forkLongitudinal, m_ForkLongitudinalAction, 0, m_ForkLongitudinalTarget, m_ForkLongitudinalSpeed); // Right arrow to extend, Left to retract
            forkLateral = GetInput(forkLateral, m_ForkLateralAction, -m_ForkLateralAdjustmentTarget, m_ForkLateralAdjustmentTarget, m_ForkLateralAdjustmentSpeed);
            forkTilt = GetInput(forkTilt, m_ForkTiltAction, -m_ForkTiltTarget, m_ForkTiltTarget, m_ForkTiltSpeed);

            var message = new MastControlMessage()
            {
                mastVerticalExtension = mastVertical,
                mastVerticalSpeed = m_MastVerticalSpeed,
                forkLongitudinalExtension = forkLongitudinal,
                forkLongitudinalSpeed = m_ForkLongitudinalSpeed,
                forkTiltTarget = forkTilt * Mathf.Deg2Rad,
                forkTiltSpeed = m_ForkTiltSpeed * Mathf.Deg2Rad,
                forkLateralExtension = forkLateral,
                forkLateralSpeed = m_ForkLateralAdjustmentSpeed
            };

            m_Controller.ConsumeMessage(message);
        }

        private float GetInput(float value, InputAction action,float min, float max, float speed)
        {
            int inputDirection = Mathf.RoundToInt(action.ReadValue<float>());

            // We interpret a speed of 0 to say that we should instantly reach the target
            if (speed == 0 && inputDirection != 0)
                if (Mathf.Sign(inputDirection) > 0)
                    return max;
                else return min;

            float input = value + inputDirection * Time.deltaTime * speed;
            input = Mathf.Clamp(input, min, max);

            return input;
        }
    }
}
