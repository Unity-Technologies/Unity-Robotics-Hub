using System;
using System.Collections.Generic;
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
    [RequireComponent(typeof(AckermannController))]
    public class AckermannAuthoring : MonoBehaviour, IAuthor
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
        [Tooltip("Stiffness is the strength of the spring that tries to reach the target position setpoint on the steering axle.")]
        [SerializeField]
        private float m_SteerStiffness = 100000f;
        [Tooltip("Damping controls the velocity-damping effect of the spring. Damping also controls how fast the ArticulationDrive tries to reach the target velocity setpoint.")]
        [SerializeField]
        private float m_SteerDamping = 10000f;

        [Header("Wheel settings")]
        [Tooltip("The mass of the driving wheels measured in kilograms")]
        [SerializeField]
        private float m_PoweredWheelMass = 15f;
        [Tooltip("The mass of the non-powered wheels measured in kilograms")]
        [SerializeField]
        private float m_NonPoweredWheelMass = 5f;
        [Tooltip("The strength of the spring that tries to reach the given target-velocity setpoint.")]
        [SerializeField]
        private float m_PoweredWheelDamping = 10000f;
        [Tooltip("The strength of the spring that tries to reach the given target-velocity setpoint. Use a non-zero value to reduce caster wheel oscillation.")]
        [SerializeField]
        private float m_NonPoweredWheelDamping = 0f;
        [Tooltip("Unity Physic Material that is used to control the friction of the wheels. Driving wheels should have higher friction.")]
        [SerializeField]
        private PhysicsMaterial m_PoweredWheelMaterial;
        [Tooltip("Unity Physic Material that is used to control the friction of the wheels. Caster wheels should have very low friction.")]
        [SerializeField]
        private PhysicsMaterial m_NonPoweredWheelMaterial;

        private AckermannController m_Controller;

        void Start()
        {
            m_Controller = GetComponent<AckermannController>();
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
            m_Controller ??= GetComponent<AckermannController>();

            if (!m_Controller)
            {
                Debug.LogError("AckermannController not found!");
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

            m_Controller.PopulateAxles();

            if (m_Controller.Axles.Count == 0)
                Debug.LogWarning("The Ackermann Controller has no assigned axles. The vehicle's wheels will not be set up.", m_Controller);

            foreach (var axle in m_Controller.Axles)
                SetupAxle(axle);
        }

        private void SetupAxle(AckermannAxle axle)
        {
            IsValidSetup(axle);

            SetupWheel(axle.LeftWheel, axle.IsPowered);
            SetupWheel(axle.RightWheel, axle.IsPowered);

            if(axle.IsSteered)
            {
                axle.LeftArm.SetXDriveStiffness(m_SteerStiffness);
                axle.LeftArm.SetXDriveDamping(m_SteerDamping);
                axle.RightArm.SetXDriveStiffness(m_SteerStiffness);
                axle.RightArm.SetXDriveDamping(m_SteerDamping);
            }
        }

        private void SetupWheel(ArticulationBody ab, bool isPowered)
        {
            var colliders = ab.GetComponentsInChildren<Collider>().Where(a => a.enabled);

            if (colliders.Count() == 0)
            {
                Debug.LogError($"This wheel ({ab.name}) has no Collider components attached. Because of that, it won't be able to collide with the ground.");
                return;
            }

            foreach (var collider in colliders)
            {
                collider.material = isPowered ? m_PoweredWheelMaterial : m_NonPoweredWheelMaterial;
            }

            if (!isPowered)
            {
                ab.mass = m_NonPoweredWheelMass;
                ab.SetXDriveStiffness(0f);
                ab.SetXDriveDamping(m_NonPoweredWheelDamping);
            }
            else
            {
                ab.mass = m_PoweredWheelMass;
                ab.SetXDriveStiffness(0f);
                ab.SetXDriveDamping(m_PoweredWheelDamping);
            }
        }

        private static bool IsValidSetup(AckermannAxle axle)
        {
            if (axle.LeftWheel == null || axle.RightWheel == null)
            {
                Debug.LogError("Either the Left or the Right wheel is null, which is not allowed", axle);
                return false;
            }

            if (axle.IsSteered)
            {
                if (axle.LeftArm == null || axle.RightArm == null)
                {
                    Debug.LogError("Left and Right arms can't be null for steered axles", axle);
                    return false;
                }

                if (axle.LeftWheel.GetParent() != axle.LeftArm || axle.RightWheel.GetParent() != axle.RightArm)
                {
                    Debug.LogError("Each wheel should be a child of its axle arm");
                    return false;
                }
            }
            else
            {
                if (axle.LeftArm != null || axle.RightArm != null)
                    Debug.LogWarning("Either left or right arm is set for the axle. Arms are ignored for non-steered axles", axle);
            }

            return true;
        }

        public static void FindAndAssignAxleComponents(AckermannController controller, AckermannAxle axle)
        {
            var worldForward = controller.WorldForward;
            var worldUp = controller.worldUp;
            var vehicleCenter = controller.Root.transform.position;

            var rightDirection = Vector3.Cross(worldForward, worldUp);

            var bodies = axle.GetComponentsInChildren<ArticulationBody>();

            if (bodies.Length == 4)
            {
                List<ArticulationBody> leftSide = new List<ArticulationBody>(2);
                List<ArticulationBody> rightSide = new List<ArticulationBody>(2);

                for(int i = 0; i < bodies.Length; i++)
                {
                    var centerToAb = vehicleCenter - bodies[i].transform.position;

                    if (Vector3.Dot(centerToAb, rightDirection) < 0)
                        leftSide.Add(bodies[i]);
                    else
                        rightSide.Add(bodies[i]);
                }

                Debug.Assert(leftSide.Count == 2, "Expected to find 2 Articulation Bodies on the left side");
                Debug.Assert(rightSide.Count == 2, "Expected to find 2 Articualtion Bodies on the right side");

                if(leftSide[0].GetParent() == leftSide[1])
                {
                    axle.LeftArm = leftSide[1];
                    axle.LeftWheel = leftSide[0];
                }
                else
                {
                    axle.LeftArm = leftSide[0];
                    axle.LeftWheel = leftSide[1];
                }

                if (rightSide[0].GetParent() == rightSide[1])
                {
                    axle.RightArm = rightSide[1];
                    axle.RightWheel = rightSide[0];
                }
                else
                {
                    axle.RightArm = rightSide[0];
                    axle.RightWheel = rightSide[1];
                }

                axle.IsSteered = true;
            }
            else if (bodies.Length == 2)
            {
                var centerToAb = vehicleCenter - bodies[0].transform.position;
                if (Vector3.Dot(centerToAb, rightDirection) < 0)
                {
                    axle.LeftWheel = bodies[0];
                    axle.RightWheel = bodies[1];
                }
                else
                {
                    axle.LeftWheel = bodies[1];
                    axle.RightWheel = bodies[0];
                }

                axle.IsSteered = false;
            }
            else
            {
                Debug.LogError($"Found {bodies.Length} Articulation Bodies under the GameObject with the {nameof(AckermannAxle)} component. The expected count is 4 for steered axles and 2 for non-steered. No Articulation Bodies will be assigned.", axle);
                return;
            }
        }
    }
}
