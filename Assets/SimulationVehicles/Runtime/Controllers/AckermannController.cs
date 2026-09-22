using System;
using UnityEngine;
using System.Collections.Generic;

namespace Unity.Simulation.VehicleControllers
{
    //                        0->   ▓═════╦═════▓ ─┐
    //                            ┌       ▒        │
    //                        1-> │ ▓═════╬═════▓  │
    //                            │       ║        │
    //                            │       ║        │
    //        relativeWheelbase > │       ║        │ L (Wheelbase)
    //        (for axle [2])      │       ║        │
    //                            │       ║        │
    //                            │       ║        │
    //                        2-> └ ▓═════╬═════▓  │
    //                              ^     ║     ^  │
    //                              α1    ║     β1 │
    //                                    ║        │
    //                        3->   ▓═════░═════▓ ─┘
    //                              ^     ^     ^
    //                              α0    γ     β0
    //                              └───────────┘
    //                                    T (TrackWidth)
    //
    // Legend:
    //    relativeWheelbase - parameterized scalar 't' along the vehicle spine from the support point to axle in question
    //    ░ - the most extreme steered axle along the vehicle spine
    //    ▒ - support point, average non-steered axle position used for base turn radius calculation
    //    vehicle's spine - an imaginary ray orignating from the root Articulation Body with the LocalForward being its direction.
    //            All the axles are parameterized along it as moentioned above

    /// <summary>
    /// This script controls the ArticulationBody components for the Ackermann Drive
    /// type vehicle. It accepts an AckermannControlMessage and converts that into
    /// ArticulationBody drive values. 
    /// </summary>
    public class AckermannController : MonoBehaviour, IRobotController<AckermannControlMessage>
    {
        [Tooltip("The first Articulation Body component in your chain.")]
        [SerializeField] private ArticulationBody m_Root;

        [Tooltip("An array of Ackermann Axle components")]
        [SerializeField] private AckermannAxle[] m_Axles;

        [Tooltip("A direction vector that specifies which way the vehicle is facing. Meaning driving forward will move the vehicle in that direction.")]
        [SerializeField] private Vector3 m_LocalForwardDirection = new Vector3(-1f, 0f, 0f);
        [Tooltip("A direction vector that specifies which way is up for the vehicle. Only used to determine which wheels are the right or left ones when trying to automatically assign axle components.")]
        [SerializeField] private Vector3 m_LocalUpDirection = new Vector3(0f, 1f, 0f);

        [SerializeField, HideInInspector] // To prevent the vehicle from losing the state on runtime domain reloads etc.
        private Data m_VehicleData;

        private AckermannControlMessage m_LastMessage;

        private float m_CurrentTargetSteeringAngle = 0f;
        private float m_CurrentTargetVelocity = 0f;
        private float m_CurrentTargetAcceleration = 0f;

        private bool m_IsSleeping = false;

        #region Public API
        /// <summary>
        /// Root Articulation Body
        /// </summary>
        public ArticulationBody Root { get { return m_Root; } set { m_Root = value; } }

        /// <summary>
        /// Read-only collection of all Axles that have been found under the controller. To refresh this list call <see cref="PopulateAxles"/>
        /// </summary>
        public IReadOnlyCollection<AckermannAxle> Axles => m_Axles;

        /// <summary>
        /// Last message received by this controller. If the <see cref="IsSleeping"/> property is false, it's still being acted upon.
        /// </summary>
        public AckermannControlMessage LastMessage => m_LastMessage;
        /// <summary>
        /// Interpolated linear velocity that was last applied to the vehicle
        /// </summary>
        public float CurrentTargetLinearVelocity => m_CurrentTargetVelocity;
        /// <summary>
        /// Interpolated linear acceleration that's used to interpolate <see cref="CurrentTargetLinearVelocity"/>
        /// </summary>
        public float CurrentTargetLinearAcceleration => m_CurrentTargetAcceleration;
        /// <summary>
        /// Interpolated steering angle that was last applied to the vehicle.
        /// </summary>
        public float CurrentTargetSteeringAngle => m_CurrentTargetSteeringAngle;

        /// <summary>
        /// Local in the <see cref="Root"/> coordinate space
        /// </summary>
        public Vector3 LocalForward { get { return m_LocalForwardDirection; } set { m_LocalForwardDirection = value; } }
        /// <summary>
        /// Local in the <see cref="Root"/> coordinate space
        /// </summary>
        public Vector3 LocalUp { get { return m_LocalUpDirection; } set { m_LocalUpDirection = value; } }

        /// <summary>
        /// <see cref="LocalForward" transformed to world coordinates/>
        /// </summary>
        public Vector3 WorldForward => m_Root.transform.TransformDirection(m_LocalForwardDirection);
        /// <summary>
        /// <see cref="LocalUp" transformed to world coordinates/>
        /// </summary>
        public Vector3 worldUp => m_Root.transform.TransformDirection(m_LocalUpDirection);

        /// <summary>
        /// Data specifying how the axles are parameterized along the vehicle's spine
        /// </summary>
        public Data VehicleData => m_VehicleData;
        /// <summary>
        /// Whether the controller is still executing the last control message or not.
        /// </summary>
        public bool IsSleeping => m_IsSleeping;

        /// <summary>
        /// This method updates the last message received in the controller.
        /// </summary>
        public void ConsumeMessage(AckermannControlMessage message)
        {
            m_IsSleeping = false;
            m_LastMessage = message;
        }

        /// <summary>
        /// Get the radius of the circle that the vehicle is travelling around.
        /// </summary>
        public float GetBaseTurnRadius()
        {
            return AckermannUtilities.GetBaseRadiusFromGamma(m_CurrentTargetSteeringAngle, m_VehicleData.EffectiveWheelbase);
        }

        /// <summary>
        /// Get the center of the circle that the vehicle is travelling around.
        /// </summary>
        public Vector3 GetCircleCenter()
        {
            var worldNonSteered = m_Root.transform.TransformPoint(m_LocalForwardDirection * m_VehicleData.SupportPoint);

            var radius = GetBaseTurnRadius();
            var direction = Mathf.Sign(m_CurrentTargetSteeringAngle) * -Vector3.Cross(WorldForward, Vector3.up);
            return worldNonSteered + direction * radius;
        }
        /// <summary>
        /// A serialized structure that holds data about the Ackermann vehicle so that
        /// the data isn't lost between domain reloads. 
        /// </summary>
        [Serializable]
        public struct Data
        {
            private float m_Wheelbase;
            private float m_TheMostExtremeSteeredAxlePosition; // Local to root
            private float m_SupportPoint; // Local to root

            /// <summary>
            /// Absolute wheelbase of the vehicle
            /// </summary>
            public float Wheelbase
            {
                get { return m_Wheelbase; }
                internal set { m_Wheelbase = value; }
            }
            /// <summary>
            /// Parameter of the axle that is furthest away. Sign is preserverd but it's direction agnostic
            /// </summary>
            public float TheMostExtremeSteeredAxlePosition
            {
                get { return m_TheMostExtremeSteeredAxlePosition; }
                internal set { m_TheMostExtremeSteeredAxlePosition = value; }
            }
            /// <summary>
            /// The aveare 't' of non-steered axles
            /// </summary>
            public float SupportPoint
            {
                get { return m_SupportPoint; }
                internal set { m_SupportPoint = value; }
            }
            /// <summary>
            /// Effective wheelbase of the vehicle. | <see cref="TheMostExtremeSteeredAxlePosition"/> - <see cref="SupportPoint"/> |
            /// </summary>
            public float EffectiveWheelbase => Mathf.Abs(m_TheMostExtremeSteeredAxlePosition - m_SupportPoint);
        }

        /// <summary>
        /// Initialize the vehicle by populating the Axles and initializing the data structure
        /// </summary>
        public void Init()
        {
            PopulateAxles();
            InitVehicleData();
        }

        /// <summary>
        /// Fill the axles array with all the axles that are children of this controller.
        /// Call this function if the amount of axles changed during runtime.
        /// </summary>
        public void PopulateAxles()
        {
            m_Axles = GetComponentsInChildren<AckermannAxle>();
        }

        #endregion

        private void Start()
        {
            Init();
        }

        private void FixedUpdate()
        {
            if (m_IsSleeping)
                return;

            InterpolateTargets(Time.fixedDeltaTime);

            var baseRadius = GetBaseTurnRadius();

            for(int i = 0; i < m_Axles.Length; i++)
                m_Axles[i].SetAxleRotationAndSpeed(baseRadius, m_CurrentTargetSteeringAngle, m_CurrentTargetVelocity);
        }

        private void InterpolateTargets(float dt)
        {
            m_CurrentTargetSteeringAngle = MathUtilities.LinearlyInterpolateFloat(m_CurrentTargetSteeringAngle, m_LastMessage.steeringAngleVelocity, m_LastMessage.steeringAngle, dt);
            m_CurrentTargetAcceleration = MathUtilities.LinearlyInterpolateFloat(m_CurrentTargetAcceleration, m_LastMessage.jerk, m_LastMessage.acceleration, dt);
            m_CurrentTargetVelocity = MathUtilities.LinearlyInterpolateFloat(m_CurrentTargetVelocity, m_CurrentTargetAcceleration, m_LastMessage.speed, dt);

            // If both of the final parameters have been reached, fall asleep
            if (Mathf.Abs(m_CurrentTargetSteeringAngle - m_LastMessage.steeringAngle) <= MathUtilities.c_Epsilon
                && Mathf.Abs(m_CurrentTargetVelocity - m_LastMessage.speed) <= MathUtilities.c_Epsilon)
                m_IsSleeping = true;
        }

        private void InitVehicleData()
        {
            m_LocalForwardDirection.Normalize();
            m_LocalUpDirection.Normalize();
            m_VehicleData = default;

            m_VehicleData.TheMostExtremeSteeredAxlePosition = 0f;
            m_VehicleData.SupportPoint = 0f;

            int nbNonSteeredAxles = 0;

            float maxForward = float.MinValue;
            float minForward = float.MaxValue;

            foreach (var axle in m_Axles)
            {
                var projectedMidpoint = axle.GetParameterAlongSpine();

                if (!axle.IsSteered)
                {
                    m_VehicleData.SupportPoint += projectedMidpoint;
                    nbNonSteeredAxles++;
                }
                else if (Mathf.Abs(projectedMidpoint) > Mathf.Abs(m_VehicleData.TheMostExtremeSteeredAxlePosition))
                    m_VehicleData.TheMostExtremeSteeredAxlePosition = projectedMidpoint;

                maxForward = Mathf.Max(maxForward, projectedMidpoint);
                minForward = Mathf.Min(minForward, projectedMidpoint);
            }

            m_VehicleData.SupportPoint /= nbNonSteeredAxles;

            m_VehicleData.Wheelbase = maxForward - minForward;
        }
    }
}
