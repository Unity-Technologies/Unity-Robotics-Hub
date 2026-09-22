using System;
using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    public class AckermannAxle : MonoBehaviour
    {
        [HideInInspector]
        [SerializeField] private AckermannController m_Controller;
        [Tooltip("Reference to the left wheel Articualtion Body")]
        [SerializeField] private ArticulationBody m_LeftWheel;
        [Tooltip("Reference to the right wheel Articualtion Body")]
        [SerializeField] private ArticulationBody m_RightWheel;
        [Tooltip("Reference to the left arm Articualtion Body")]
        [SerializeField] private ArticulationBody m_LeftArm;
        [Tooltip("Reference to the right arm Articualtion Body")]
        [SerializeField] private ArticulationBody m_RightArm;
        [Tooltip("Whether this axle is steerable or not")]
        [SerializeField] private bool m_IsSteered;
        [Tooltip("Whether this axle is powered or not")]
        [SerializeField] private bool m_IsPowered;

        [NonSerialized] private float m_LeftAngle;
        [NonSerialized] private float m_RightAngle;
        [NonSerialized] private float m_LeftWheelRadiusCache;
        [NonSerialized] private float m_RightWheelRadiusCache;

        /// <summary>
        /// <see cref="AckermannController"/> that supplies steering angles and linear velocity to this axle
        /// </summary>
        public AckermannController Controller { get { return m_Controller; } set { m_Controller = value; } }
        public ArticulationBody LeftWheel { get { return m_LeftWheel; } set { m_LeftWheel = value; } }
        public ArticulationBody RightWheel { get { return m_RightWheel; } set { m_RightWheel = value; } }
        public ArticulationBody LeftArm { get { return m_LeftArm; } set { m_LeftArm = value; } }
        public ArticulationBody RightArm { get { return m_RightArm; } set { m_RightArm = value; } }

        public bool IsSteered { get { return m_IsSteered; } set { m_IsSteered = value; } }
        public bool IsPowered { get { return m_IsPowered; } set { m_IsPowered = value; } }

        /// <summary>
        /// Track width of this axle. Measured between the left and right wheels
        /// </summary>
        public float TrackWidth => (m_LeftWheel.transform.position - m_RightWheel.transform.position).magnitude;
        /// <summary>
        /// Radius of the left wheel <see cref="Collider"/>
        /// </summary>
        public float LeftWheelRadius => GetLeftWheelRadius();
        /// <summary>
        /// Radius of the right wheel <see cref="Collider"/>
        /// </summary>
        public float RightWheelRadius => GetRightWheelRadius();
        /// <summary>
        /// Mid-point of the axle in world coordinates
        /// </summary>
        public Vector3 WorldMidPoint => (m_LeftWheel.transform.position + m_RightWheel.transform.position) / 2f;

        private void Awake()
        {
            m_Controller = GetComponentInParent<AckermannController>();

            if (!m_Controller)
                throw new NotSupportedException($"{nameof(AckermannController)} must be a parent of {nameof(AckermannAxle)}");
        }

        /// <summary>
        /// Calculates the 't' scalar of this axle along the vehicle's spine
        /// </summary>
        public float GetParameterAlongSpine()
        {
            var localMidpoint = m_Controller.Root.transform.InverseTransformPoint(WorldMidPoint);
            return Vector3.Dot(m_Controller.LocalForward, localMidpoint);
        }

        /// <summary>
        /// Calculates and sets the axle arm rotation (if <see cref="IsSteered"/> is <see langword="true"/>) and the wheel linear velocity (if <see cref="IsPowered"/> is <see langword="true"/>)
        /// </summary>
        /// <param name="baseRadius">Base turn radius of the vehicle</param>
        /// <param name="gamma">Turn angle of the vehicle</param>
        /// <param name="linearVel">Linear velocity which the vehicle should move at</param>
        /// <exception cref="NullReferenceException"></exception>
        public void SetAxleRotationAndSpeed(float baseRadius, float gamma, float linearVel)
        {
            if(m_IsSteered)
            {
                var distanceBetweenNonSteeredAndThis = GetParameterAlongSpine() - m_Controller.VehicleData.SupportPoint;

                var (leftAngle, rightAngle) = AckermannUtilities.GetWheelAnglesRad(baseRadius, gamma, distanceBetweenNonSteeredAndThis, TrackWidth); // In radians

                SetAxleRotationRad(leftAngle, rightAngle);
            }

            if(m_IsPowered)
            {
                var targetDPS = AckermannUtilities.LinearSpeedToDegreesPerSecond(linearVel, LeftWheelRadius);

                m_LeftWheel.SetXDriveTargetVelocity(targetDPS);
                m_RightWheel.SetXDriveTargetVelocity(targetDPS);
            }
        }

        /// <summary>
        /// Directly sets left and right arm angles in radians
        /// </summary>
        /// <param name="leftAngle">Left arm turn angle in radians</param>
        /// <param name="rightAngle">Right arm turn angle in radians</param>
        /// <exception cref="NullReferenceException"></exception>
        public void SetAxleRotationRad(float leftAngle, float rightAngle)
        {
            m_LeftArm.SetXDriveTarget(leftAngle * Mathf.Rad2Deg); // These need deg
            m_RightArm.SetXDriveTarget(rightAngle * Mathf.Rad2Deg);
            m_LeftAngle = leftAngle;
            m_RightAngle = rightAngle;
        }

        /// <summary>
        /// Directly sets left and right arm angles in degrees
        /// </summary>
        /// <param name="leftAngle">Left arm turn angle in degrees</param>
        /// <param name="rightAngle">Right arm turn angle in degrees</param>
        /// <exception cref="NullReferenceException"></exception>
        public void SetAxleRotationDeg(float leftAngle, float rightAngle)
        {
            m_LeftArm.SetXDriveTarget(leftAngle);
            m_RightArm.SetXDriveTarget(rightAngle);
            m_LeftAngle = leftAngle * Mathf.Deg2Rad;
            m_RightAngle = rightAngle * Mathf.Deg2Rad;
        }

        public override bool Equals(object obj)
        {
            return obj is AckermannAxle axle && Equals(axle);
        }

        /// <summary>
        /// Compares two <see cref="AckermannAxle" /> components and returns true if they are identical.
        /// </summary>
        public bool Equals(AckermannAxle other)
        {
            return EqualityComparer<ArticulationBody>.Default.Equals(m_LeftWheel, other.m_LeftWheel) &&
                   EqualityComparer<ArticulationBody>.Default.Equals(m_RightWheel, other.m_RightWheel) &&
                   EqualityComparer<ArticulationBody>.Default.Equals(m_LeftArm, other.m_LeftArm) &&
                   EqualityComparer<ArticulationBody>.Default.Equals(m_RightArm, other.m_RightArm) &&
                   m_IsSteered == other.m_IsSteered &&
                   m_IsPowered == other.m_IsPowered;
        }

        /// <summary>
        /// Gets a semi-random hash value for colouring the debug trajectories.
        /// </summary>
        public override int GetHashCode()
        {
            unchecked
            {
                int hash = 17;
                hash = hash * 31 + m_LeftWheel.GetHashCode();
                hash = hash * 31 + m_RightWheel.GetHashCode();
                hash = hash * 31 + m_LeftArm.GetHashCode();
                hash = hash * 31 + m_RightArm.GetHashCode();
                hash = hash * 31 + m_IsSteered.GetHashCode();
                hash = hash * 31 + m_IsPowered.GetHashCode();
                return hash;
            }
        }

        private float GetLeftWheelRadius()
        {
            if (m_LeftWheelRadiusCache == 0f)
                m_LeftWheelRadiusCache = CalculateWheelRadiusForArticulation(m_LeftWheel);

            return m_LeftWheelRadiusCache;
        }

        private float GetRightWheelRadius()
        {
            if (m_RightWheelRadiusCache == 0f)
                m_RightWheelRadiusCache = CalculateWheelRadiusForArticulation(m_RightWheel);

            return m_RightWheelRadiusCache;
        }

        private float CalculateWheelRadiusForArticulation(ArticulationBody ab)
        {
            var colliders = ab.GetComponentsInChildren<Collider>().Where(a => a.enabled);

            if (colliders.Count() == 0)
            {
                Debug.LogError("Wheel ArticulationBody has no collider!");
                return 0.5f;
            }

            var bounds = colliders.First().bounds;
            Debug.Assert(0f != bounds.extents.y, "Wheel radius was 0f!");
            return bounds.extents.y;
        }
    }
}
