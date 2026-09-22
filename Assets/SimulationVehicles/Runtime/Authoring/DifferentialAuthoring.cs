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
    [RequireComponent(typeof(DifferentialDriveController))]
    public class DifferentialAuthoring : MonoBehaviour, IAuthor
    {
        [Header("Articulation bodies")]
        [Tooltip("The first ArticulationBody component in your chain.")]
        [SerializeField]
        private ArticulationBody m_ArticulationRoot;
        [Tooltip("These can be any amount of caster wheels. They will not be participating in driving or steering the vehicle, but will only help support the vehicle.")]
        [SerializeField]
        private ArticulationBody[] m_CasterWheels;

        [Header("Body settings")]
        [Tooltip("The mass of the vehicle’s root ArticulationBody measured in kilograms.")]
        [SerializeField]
        private float m_VehicleBodyMass = 50f;
        [Tooltip("When set to Automatic, the Center of Mass will be calculated based on the root ArticulationBody and its colliders.")]
        [SerializeField]
        private bool m_AutomaticCenterOfMass = true;
        [SerializeField]
        private Vector3 m_CenterOfMass = Vector3.zero;

        [Header("Axle settings")]
        [Tooltip("The strength of the spring that tries to reach the given target-velocity setpoint. This will set the Damping term on both driving wheel ArticulationDrives.")]
        [SerializeField]
        private float m_MotorStrength = 10000f;
        [Tooltip("The mass of the driving wheels measured in kilograms")]
        [SerializeField]
        private float m_DrivingWheelMass = 10f;
        [Tooltip("The mass of the caster wheels measured in kilograms")]
        [SerializeField]
        private float m_CasterWheelMass = 5f;
        [Tooltip("The strength of the spring that tries to reach the given target-velocity setpoint. Use a non-zero value to reduce caster wheel oscillation.")]
        [SerializeField]
        private float m_CasterWheelDamping = 50f;
        [Tooltip("Unity Physic Material that is used to control the friction of the wheels. Driving wheels should have higher friction.")]
        [SerializeField]
        private PhysicsMaterial m_DrivingWheelMaterial;
        [Tooltip("Unity Physic Material that is used to control the friction of the wheels. Caster wheels should have very slippery friction.")]
        [SerializeField]
        private PhysicsMaterial m_CasterWheelMaterial;
        [SerializeField]
        private DifferentialDriveController m_Controller;
        [Tooltip("Whether to disable the collision between the different parts of the robot. Useful when some parts are overlapping each other and causing simulation instabilities.")]
        [SerializeField]
        private bool m_DisableInterCollision = true;

        void Start()
        {
            m_Controller = GetComponent<DifferentialDriveController>();
            if (m_DisableInterCollision)
            {
                DisableInterCollision();
            }
        }

        void Reset()
        {
            SetupArticulations();
        }

        private void DisableInterCollision()
        {
            Collider[] colliders = m_ArticulationRoot.GetComponentsInChildren<Collider>();

            int len = colliders.Length;

            for (int i = 0; i < len - 1; i++)
            {
                for (int j = i; j < len; j++)
                {
                    Physics.IgnoreCollision(colliders[i], colliders[j], true);
                }
            }

        }

        /// <summary>
        /// This method propagates all the values from the script to the
        /// ArticulationBody components
        /// </summary>
        [ContextMenu("Execute Setup")]
        public void SetupArticulations()
        {
            m_Controller ??= GetComponent<DifferentialDriveController>();

            if (!m_Controller)
            {
                Debug.LogError("DifferentialDriveController not found!");
                return;
            }

            if (m_ArticulationRoot)
            {
                m_ArticulationRoot.mass = m_VehicleBodyMass;
                if (m_AutomaticCenterOfMass)
                    m_ArticulationRoot.ResetCenterOfMass();
                else
                    m_ArticulationRoot.centerOfMass = m_CenterOfMass;
            }
            else
                Debug.LogWarning("Root articulation body is not assigned! Default values will be applied once the root is assigned.");

            if (m_Controller.WheelPairs.Length > 0)
            {
                foreach (var wheelPair in m_Controller.WheelPairs)
                {
                    SetupWheel(wheelPair.leftWheel, false);
                    SetupWheel(wheelPair.rightWheel, false);
                }
            }
            else
                Debug.LogWarning("Wheel pairs are not assigned! Make sure to set up the Controller component as the Authoring script takes references from it.");

            if (m_CasterWheels.Length > 0)
            {
                foreach (var caster in m_CasterWheels)
                {
                    SetupWheel(caster, true);
                }
            }
            else
                Debug.LogWarning("Caster wheels are not assigned! Default values will be applied once the wheels are assigned.");
        }

        private void SetupWheel(ArticulationBody ab, bool isCaster)
        {
            var colliders = ab.GetComponentsInChildren<Collider>().Where(a => a.enabled);

            if (colliders.Count() == 0)
            {
                Debug.LogError($"This wheel ({ab.name}) has no Collider components attached. Because of that, it won't be able to collide with the ground.");
                return;
            }

            foreach (var collider in colliders)
            {
                collider.material = isCaster ? m_CasterWheelMaterial : m_DrivingWheelMaterial;
            }

            if (!isCaster)
            {
                ab.mass = m_DrivingWheelMass;
                ab.SetXDriveDamping(m_MotorStrength);
            }
            else
            {
                ab.mass = m_CasterWheelMass;
                ab.SetXDriveDamping(m_CasterWheelDamping);
            }
        }
    }
}
