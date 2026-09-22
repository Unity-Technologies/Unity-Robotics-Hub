using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This Authoring script propagates values to the relevant ArticulationBody
    /// components. Changes are propagated automatically, but if that should fail,
    /// right click the component in the inspector and select "Execute Setup"
    /// </summary>
    [RequireComponent(typeof(ThreeWheelController))]
    public class ThreeWheelAuthoring : MonoBehaviour, IAuthor
    {
        [Header("Body settings")]
        [Tooltip("The mass of the vehicle’s root ArticulationBody measured in kilograms.")]
        [SerializeField]
        private float m_VehicleBodyMass = 500f;
        [Tooltip("When set to Automatic, the Center of Mass will be calculated based on the root ArticulationBody and its colliders.")]
        [SerializeField]
        private bool m_UseAutomaticCenterOfMass = true;
        [SerializeField]
        private Vector3 m_CenterOfMass = Vector3.zero;

        [Header("Axle settings")]
        [Tooltip("Strength of the spring that tries to reach the target position on the steering axle.")]
        [SerializeField]
        private float m_SteerStiffness = 100000f;
        [Tooltip("Controls how fast the ArticulationDrive tries to reach the target velocity setpoint.")]
        [SerializeField]
        private float m_SteerDamping = 10000f;

        [Header("Wheel settings")]
        [Tooltip("The mass of the driving wheels measured in kilograms")]
        [SerializeField]
        private float m_DrivingWheelMass = 30f;
        [Tooltip("The mass of the caster wheels measured in kilograms")]
        [SerializeField]
        private float m_CasterWheelMass = 5f;
        [Tooltip("The strength of the spring that tries to reach the given target-velocity setpoint.")]
        [SerializeField]
        private float m_DrivingWheelDamping = 10000f;
        [Tooltip("The strength of the spring that tries to reach the given target-velocity setpoint. Use a non-zero value to reduce caster wheel oscillation.")]
        [SerializeField]
        private float m_CasterWheelDamping = 20f;
        [Tooltip("Unity Physic Material that is used to control the friction of the wheels. Driving wheels should have higher friction.")]
        [SerializeField]
        private PhysicsMaterial m_DrivingWheelMaterial;
        [Tooltip("Unity Physic Material that is used to control the friction of the wheels. Caster wheels should have very slippery friction.")]
        [SerializeField]
        private PhysicsMaterial m_CasterWheelMaterial;

        private ThreeWheelController m_Controller;

        void Start()
        {
            m_Controller = GetComponent<ThreeWheelController>();
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
            m_Controller ??= GetComponent<ThreeWheelController>();

            if (!m_Controller)
            {
                Debug.LogError("ThreeWheelController not found!");
                return;
            }

            if (m_Controller.Root)
            {
                m_Controller.Root.mass = m_VehicleBodyMass;

                if (m_UseAutomaticCenterOfMass)
                    m_Controller.Root.ResetCenterOfMass();
                else
                    m_Controller.Root.centerOfMass = m_CenterOfMass;
            }
            else
                Debug.LogWarning("Root articulation body is not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");


            if (m_Controller.FrontWheels.Length > 0)
            {
                foreach (var pair in m_Controller.FrontWheels)
                {
                    SetupWheel(pair.leftWheel, true);
                    SetupWheel(pair.rightWheel, true);
                }
            }
            else
                Debug.LogWarning("Front wheels are not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");

            if (m_Controller.DrivingWheel)
                SetupWheel(m_Controller.DrivingWheel, false);
            else
                Debug.LogWarning("Driving wheel is not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");

            if (m_Controller.SteerAxle)
            {
                m_Controller.SteerAxle.SetXDriveStiffness(m_SteerStiffness);
                m_Controller.SteerAxle.SetXDriveDamping(m_SteerDamping);
            }
            else
                Debug.LogWarning("Steer axle body is not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");
        }

        private void SetupWheel(ArticulationBody ab, bool isCasterWheel)
        {
            var colliders = ab.GetComponentsInChildren<Collider>().Where(a => a.enabled);

            if (colliders.Count() == 0)
            {
                Debug.LogError($"This wheel ({ab.name}) has no Collider components attached. Because of that, it won't be able to collide with the ground.");
                return;
            }

            foreach (var collider in colliders)
            {
                collider.material = isCasterWheel ? m_CasterWheelMaterial : m_DrivingWheelMaterial;
            }

            if (!isCasterWheel)
            {
                ab.mass = m_DrivingWheelMass;
                ab.SetXDriveStiffness(0f);
                ab.SetXDriveDamping(m_DrivingWheelDamping);
            }
            else
            {
                ab.mass = m_CasterWheelMass;
                ab.SetXDriveStiffness(0f);
                ab.SetXDriveDamping(m_CasterWheelDamping);
            }
        }
    }
}
