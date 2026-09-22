using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    public class MastController : MonoBehaviour, IRobotController<MastControlMessage>
    {
        [Tooltip("These are ArticulationBodies with prismatic joints that have their degree of freedom in the vertical direction.")]
        [SerializeField] private List<ArticulationBody> m_MastLevels;

        [Tooltip("ArticulationBody that controls longitudinal extension of the fork.")]
        [SerializeField] private ArticulationBody m_ForkLongitudinalArticulation;

        [Tooltip("ArticulationBody that controls tilting the carriage backwards.")]
        [SerializeField] private ArticulationBody m_ForkTilterArticulation;

        [Tooltip("ArticulationBody that controls sideways movement of the fork.")]
        [SerializeField] private ArticulationBody m_ForkLateralArticulation;

        // Private controller state
        private MastControlMessage m_LastMessage;

        private float m_CurrentTargetMastVerticalExtension = 0f; // m
        private float m_CurrentTargetForkLongitudinalExtension = 0f; //m
        private float m_CurrentForkTilt = 0f; //Rad
        private float m_CurrentForkLateralExtension = 0f;
        private bool m_IsSleeping = false;

        // Public properties, must call the SetupArticulations() method after any of the setters

        /// <summary>
        /// The levels of the elevator/mast. The elevator can be constructed from one contiguous piece
        /// or it can be made up out of multiple levels. In case of the latter, they will still
        /// function the same as a single level, but each individual height will be distributed into smaller pieces.
        /// </summary>
        public List<ArticulationBody> MastLevels { get { return m_MastLevels; } set { m_MastLevels = value; } }
        /// <summary>
        /// The ArticulationBody component that is responsible for Longitudinal motion. (i.e. forward/backward)
        /// </summary>
        public ArticulationBody ForkLongitudinalArticulation { get { return m_ForkLongitudinalArticulation; } set { m_ForkLongitudinalArticulation = value; } }
        /// <summary>
        /// The ArticulationBody component that is responsible for a tilting motion (i.e. rotation along the lateral axis).
        /// </summary>
        public ArticulationBody ForkTilterArticulation { get { return m_ForkTilterArticulation; } set { m_ForkTilterArticulation = value; } }
        /// <summary>
        /// The ArticulationBody component that is responsible for Lateral motion. (i.e. left/right)
        /// </summary>
        public ArticulationBody ForkLateralArticulation { get { return m_ForkLateralArticulation; } set { m_ForkLateralArticulation = value; } }

        /// <summary>
        /// Last message received by this controller.
        /// </summary>
        public MastControlMessage LastMessage => m_LastMessage;

        /// <summary>
        /// Interpolated mast vertical extension that was last applied to the vehicle.
        /// </summary>
        public float CurrentTargetMastVerticalExtension => m_CurrentTargetMastVerticalExtension;
        /// <summary>
        /// Interpolated fork longitudinal extension that was last applied to the vehicle.
        /// </summary>
        public float CurrentTargetForkLongitudinalExtension => m_CurrentTargetForkLongitudinalExtension;
        /// <summary>
        /// Interpolated fork tilt that was last applied to the vehicle.
        /// </summary>
        public float CurrentForkTilt => m_CurrentForkTilt;
        /// <summary>
        /// Interpolated fork lateral extension that was last applied to the vehicle.
        /// </summary>
        public float CurrentForkLateralExtension => m_CurrentForkLateralExtension;
        /// <summary>
        /// Whether the controller is still executing the last control message or not.
        /// </summary>
        public bool IsSleeping => m_IsSleeping;

        /// <summary>
        /// This method updates the last message received in the controller.
        /// </summary>
        public void ConsumeMessage(MastControlMessage message)
        {
            m_IsSleeping = false;
            m_LastMessage = message;
        }

        private void InterpolateInput(float dt)
        {
            m_CurrentTargetMastVerticalExtension = MathUtilities.LinearlyInterpolateFloat(m_CurrentTargetMastVerticalExtension, m_LastMessage.mastVerticalSpeed, m_LastMessage.mastVerticalExtension, dt);
            m_CurrentTargetForkLongitudinalExtension = MathUtilities.LinearlyInterpolateFloat(m_CurrentTargetForkLongitudinalExtension, m_LastMessage.forkLongitudinalSpeed, m_LastMessage.forkLongitudinalExtension, dt);
            m_CurrentForkTilt = MathUtilities.LinearlyInterpolateFloat(m_CurrentForkTilt, m_LastMessage.forkTiltSpeed, m_LastMessage.forkTiltTarget, dt);
            m_CurrentForkLateralExtension = MathUtilities.LinearlyInterpolateFloat(m_CurrentForkLateralExtension, m_LastMessage.forkLateralSpeed, m_LastMessage.forkLateralExtension, dt);

            m_CurrentTargetForkLongitudinalExtension = Mathf.Clamp(m_CurrentTargetForkLongitudinalExtension, 0f, float.MaxValue); // This shouldn't go negative

            if (Mathf.Abs(m_CurrentTargetMastVerticalExtension - m_LastMessage.mastVerticalExtension) <= MathUtilities.c_Epsilon
                && Mathf.Abs(m_CurrentTargetForkLongitudinalExtension - m_LastMessage.forkLongitudinalExtension) <= MathUtilities.c_Epsilon
                && Mathf.Abs(m_CurrentForkTilt - m_LastMessage.forkTiltTarget) <= MathUtilities.c_Epsilon
                && Mathf.Abs(m_CurrentForkLateralExtension - m_LastMessage.forkLateralExtension) <= MathUtilities.c_Epsilon) // All current targets are equal to final targets
                m_IsSleeping = true;
        }

        private void FixedUpdate()
        {
            if (m_IsSleeping)
                return;

            InterpolateInput(Time.fixedDeltaTime);

            int n = m_MastLevels.Count;
            var targetHeight = m_CurrentTargetMastVerticalExtension / n;

            for (int i = 0; i < n; i++)
                m_MastLevels[i].SetXDriveTarget(targetHeight);

            if (m_ForkLongitudinalArticulation != null)
                m_ForkLongitudinalArticulation.SetXDriveTarget(m_CurrentTargetForkLongitudinalExtension);

            if (m_ForkTilterArticulation != null)
                m_ForkTilterArticulation.SetXDriveTarget(m_CurrentForkTilt * Mathf.Rad2Deg);

            if (m_ForkLateralArticulation != null)
                m_ForkLateralArticulation.SetXDriveTarget(m_CurrentForkLateralExtension);
        }
    }
}
