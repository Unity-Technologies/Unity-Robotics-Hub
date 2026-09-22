using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This Authoring script propagates values to the relevant ArticulationBody
    /// components. Changes are propagated automatically, but if that should fail,
    /// right click the component in the inspector and select "Execute Setup"
    /// </summary>
    [RequireComponent(typeof(MastController))]
    public class MastAuthoring : MonoBehaviour, IAuthor
    {
        [Header("Mast Vertical")]
        [Tooltip("Strength of the spring that controls the vertical movement of the elevator. Higher values mean a more rigid and precise movement.")]
        [SerializeField]
        private float m_MastVerticalStiffness = 1000f;
        [Tooltip("How much the velocity changes should be dampened, resulting in a slower acceleration.")]
        [SerializeField]
        private float m_MastVerticalDamping = 100f;

        [Header("Mast Longitudinal")]
        [Tooltip("How far the fork should be allowed to extend forward. Measured in meters.")]
        [SerializeField]
        private float m_MaxForkLongitudinalExtension = 1f;
        [Tooltip("Strength of the spring that controls the forward movement of the fork. Higher values mean a more rigid and precise movement.")]
        [SerializeField]
        private float m_ForkLongitudinalStiffness = 1000f;
        [Tooltip("How much the velocity changes of the fork should be dampened, resulting in a slower acceleration.")]
        [SerializeField]
        private float m_ForkLongitudinalDamping = 100f;

        [Header("Fork Tilt")]
        [Tooltip("The upper boundary of how much the tilter should be able to rotate the fork. Measured in degrees.")]
        [SerializeField]
        private float m_ForkTiltUpperLimit = 7.5f;
        [Tooltip("The lower boundary of how much the tilter should be able to rotate the fork. Measured in degrees.")]
        [SerializeField]
        private float m_ForkTiltLowerLimit = -1f;
        [Tooltip("Strength of the spring that controls the tilting of the carriage. Higher values mean a more rigid and precise movement.")]
        [SerializeField]
        private float m_ForkTiltStiffness = 1000f;
        [Tooltip("How much the velocity changes should be dampened, resulting in a slower acceleration.")]
        [SerializeField]
        private float m_ForkTiltDamping = 100f;

        [Header("Fork Lateral")]
        [Tooltip("The upper boundary of how much the sideways motion should be able to extend. Measured in meters.")]
        [SerializeField]
        private float m_ForkLateralUpperLimit = 0.12f;
        [Tooltip("The lower boundary of how much the sideways motion should be able to extend. Measured in meters.")]
        [SerializeField]
        private float m_ForkLateralLowerLimit = -0.12f;
        [Tooltip("Strength of the spring that controls the sideways fork movement. Higher values mean a more rigid and precise movement.")]
        [SerializeField]
        private float m_ForkLateralStiffness = 1000f;
        [Tooltip("How much the velocity changes should be dampened, resulting in a slower acceleration.")]
        [SerializeField]
        private float m_ForkLateralDamping = 100f;

        private MastController m_Controller;

        void Start()
        {
            m_Controller = GetComponent<MastController>();
        }

        void Reset()
        {
            SetupArticulations();
        }

        /// <summary>
        /// This method propagates all the values from the script to the
        /// ArticulationBody components
        /// </summary>
        [ContextMenu("Execute Setup")]
        public void SetupArticulations()
        {
            m_Controller ??= GetComponent<MastController>();

            if (!m_Controller)
            {
                Debug.LogError("MastController not found!");
                return;
            }

            if (m_Controller.MastLevels.Count > 0)
            {
                for (int i = 0; i < m_Controller.MastLevels.Count; i++)
                {
                    m_Controller.MastLevels[i].SetXDriveStiffness(m_MastVerticalStiffness);
                    m_Controller.MastLevels[i].SetXDriveDamping(m_MastVerticalDamping);
                }
            }
            else
                Debug.LogWarning("Mast levels are not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");
            

            if (m_Controller.ForkLongitudinalArticulation != null)
            {
                m_Controller.ForkLongitudinalArticulation.SetXDriveLowerLimit(0f);
                m_Controller.ForkLongitudinalArticulation.SetXDriveUpperLimit(m_MaxForkLongitudinalExtension);
                m_Controller.ForkLongitudinalArticulation.SetXDriveStiffness(m_ForkLongitudinalStiffness);
                m_Controller.ForkLongitudinalArticulation.SetXDriveDamping(m_ForkLongitudinalDamping);
            }
            else
                Debug.LogWarning("Fork Longitudinal body is not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");

            if (m_Controller.ForkTilterArticulation != null)
            {
                m_Controller.ForkTilterArticulation.SetXDriveUpperLimit(m_ForkTiltUpperLimit);
                m_Controller.ForkTilterArticulation.SetXDriveLowerLimit(m_ForkTiltLowerLimit);
                m_Controller.ForkTilterArticulation.SetXDriveStiffness(m_ForkTiltStiffness);
                m_Controller.ForkTilterArticulation.SetXDriveDamping(m_ForkTiltDamping);
            }
            else
                Debug.LogWarning("Fork Tilter body is not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");

            if (m_Controller.ForkLateralArticulation != null)
            {
                m_Controller.ForkLateralArticulation.SetXDriveLowerLimit(m_ForkLateralLowerLimit);
                m_Controller.ForkLateralArticulation.SetXDriveUpperLimit(m_ForkLateralUpperLimit);
                m_Controller.ForkLateralArticulation.SetXDriveStiffness(m_ForkLateralStiffness);
                m_Controller.ForkLateralArticulation.SetXDriveDamping(m_ForkLateralDamping);
            }
            else
                Debug.LogWarning("Fork Lateral body is not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");
        }
    }
}
