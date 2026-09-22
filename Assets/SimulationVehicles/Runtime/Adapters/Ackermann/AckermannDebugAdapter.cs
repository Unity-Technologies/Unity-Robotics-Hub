using System.Collections.Generic;
using UnityEngine;
#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This adapter can send fabricated input every frame and draw wheel trajectories of a turning vehicle.
    /// Note that this adapter can be attached to a parent GameObject and send input to multiple controllers.
    /// </summary>
    public class AckermannDebugAdapter : MonoBehaviour, IAckermannAdapter
    {
        [Header("Input fabrication")]
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

        [SerializeField, Range(-1f, 1f)]
        private float m_SteerInput = 0f;

        [SerializeField, Range(-1f, 1f)]
        private float m_AcceleratorInput = 0f;

        [SerializeField]
        private bool m_BrakeInput = false;

        [Header("Settings")]
        [SerializeField]
        [Tooltip("Creates and sends control messages every frame")]
        private bool m_ProvideInput = false;
        [SerializeField]
        private bool m_DrawWheelTrajectories = false;
        [SerializeField]
        private bool m_DrawDirectionArrows = false;

        private AckermannController[] m_Controllers;

        /// <summary>
        /// Whether this debug adapter provides input to all children Ackermann controllers or not. 
        /// </summary>
        public bool ProvideInput { get { return m_ProvideInput; } set { m_ProvideInput = value; } }

        /// <summary>
        /// Whether to draw the concentric circles that show the path of each wheel or not. 
        /// </summary>
        public bool DrawWheelTrajectories { get { return m_DrawWheelTrajectories; } set { m_DrawWheelTrajectories = value; } }

        /// <summary>
        /// Whether to draw the Forward and Up direction arrows in the scene view or not.
        /// </summary>
        public bool DrawDirectionArrows { get { return m_DrawDirectionArrows; } set { m_DrawDirectionArrows = value; } }

        private void Start()
        {
            m_Controllers = GetComponentsInChildren<AckermannController>();
        }

        private void Update()
        {
            if (!m_ProvideInput)
                return;

            foreach(var controller in m_Controllers)
            {
                var message = new AckermannControlMessage()
                {
                    steeringAngle = m_SteerInput * m_MaxSteeringAngle * Mathf.Deg2Rad,
                    speed = m_BrakeInput ? 0f : m_MaxLinearVelocity * m_AcceleratorInput,
                    acceleration = m_BrakeInput ? 0f : m_LinearAcceleration,
                    steeringAngleVelocity = m_SteeringVelocity * Mathf.Deg2Rad,
                    jerk = m_LinearJerk
                };

                controller.ConsumeMessage(message);
            }
        }

#if UNITY_EDITOR
        private static class Styles
        {
            public static GUIContent forward = new GUIContent("Forward", "Forward direction of the vehicle");
            public static GUIContent up = new GUIContent("Up", "Up direction of the vehicle");
            public static float offsetFromCenter = 0.5f;
            public static float arrowLength = 2f;
            public static int fontSize = 20;
        }

        private void OnDrawGizmos()
        {
            if(m_Controllers == null || m_Controllers.Length == 0)
                m_Controllers = GetComponentsInChildren<AckermannController>();

            if (m_Controllers == null)
                return;

            foreach (var controller in m_Controllers)
            {
                // Directions
                if(m_DrawDirectionArrows)
                {
                    var rootPos = controller.Root.transform.position;
                    var worldForward = controller.WorldForward;
                    var worldUp = controller.worldUp;
                    var labelStyle = new GUIStyle("label");
                    labelStyle.fontSize = Styles.fontSize;

                    Handles.color = Color.cyan;
                    Handles.ArrowHandleCap(0, rootPos + worldForward * Styles.offsetFromCenter, Quaternion.LookRotation(worldForward, worldUp), Styles.arrowLength, EventType.Repaint);
                    Handles.color = Color.white;
                    
                    Handles.Label(rootPos + worldForward * (Styles.offsetFromCenter + Styles.arrowLength), Styles.forward, labelStyle);

                    Handles.color = Color.yellow;
                    Handles.ArrowHandleCap(0, rootPos + worldUp * Styles.offsetFromCenter, Quaternion.LookRotation(worldUp, worldUp), Styles.arrowLength, EventType.Repaint);
                    Handles.color = Color.white;
                    Handles.Label(rootPos + worldUp * (Styles.offsetFromCenter + Styles.arrowLength), Styles.up, labelStyle);
                }

                if (m_DrawWheelTrajectories)
                {
                    if (Mathf.Abs(controller.CurrentTargetSteeringAngle) <= MathUtilities.c_Epsilon)
                        continue;

                    var circleCenter = controller.GetCircleCenter();

                    Handles.color = Color.magenta;
                    Handles.ConeHandleCap(0, circleCenter, Quaternion.LookRotation(new Vector3(0f, 1f, 0f)), 0.5f, EventType.Repaint);

                    foreach (var axle in controller.Axles)
                    {
                        Handles.color = MathUtilities.GetColorFromHash(axle.GetHashCode());
                        var distanceLeft = (axle.LeftWheel.transform.position - circleCenter).magnitude;
                        var distanceRight = (axle.RightWheel.transform.position - circleCenter).magnitude;
                        Handles.DrawWireDisc(circleCenter, new Vector3(0f, 1f, 0f), distanceLeft, 2f);
                        Handles.DrawWireDisc(circleCenter, new Vector3(0f, 1f, 0f), distanceRight, 2f);
                    }
                }   
            }  
        }
#endif
    }
}
