using System;
using System.Linq;
using UnityEngine;
using UnityEngine.Assertions;
using UnityEngine.UIElements;

#if UNITY_EDITOR
using UnityEditor;
#endif

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This script controls the ArticulationBody components for the Three Wheel drive
    /// type vehicle. It accepts an ThreeWheelControlMessage and converts that into
    /// ArticulationBody drive values. 
    /// </summary>
    public class ThreeWheelController : MonoBehaviour, IRobotController<ThreeWheelControlMessage>
    {
        /// <summary>
        /// A structure that holds the left and right ArticulationBody wheels.
        /// </summary>
        [Serializable]
        public struct WheelPair
        {
            public ArticulationBody leftWheel;
            public ArticulationBody rightWheel;
        }

        [Tooltip("The first ArticulationBody component in your chain — the root of the whole robot")]
        [SerializeField] private ArticulationBody m_Root;

        [Tooltip("ArticulationBody components that are responsible for turning the wheel when steering the vehicle.")]
        [SerializeField] private ArticulationBody m_SteerAxle;

        [Tooltip("These can be any amount of caster wheels added in pairs. They will not be participating in driving or steering the vehicle, but will only help support the vehicle.")]
        [SerializeField] private WheelPair[] m_FrontWheels;

        [Tooltip("This is the wheel that will be driving the vehicle.")]
        [SerializeField] private ArticulationBody m_DrivingWheel;

        [Tooltip("A direction vector that specifies which way the vehicle is facing. Meaning driving forward will move the vehicle in that direction.")]
        [SerializeField] private Vector3 m_LocalForwardDirection = new Vector3(0f, 0f, 1f); // Local direction in which the vehicle is going to drive forward

        // Private controller state
        [SerializeField, HideInInspector] // To prevent the vehicle from losing the state on runtime domain reloads etc.
        private VehicleData m_VehicleData;

        private ThreeWheelControlMessage m_LastMessage;

        private float m_CurrentTargetSteeringAngle = 0f;
        private float m_CurrentTargetVelocity = 0f;
        private float m_CurrentTargetAcceleration = 0f;

        internal float m_VelocitySign = 0f;

        private bool m_IsSleeping = false;

        // Public properties
        /// <summary>
        /// The root ArticulationBody of the robot vehicle.
        /// </summary>
        public ArticulationBody Root { get => m_Root; set => m_Root = value; }

        /// <summary>
        /// The steering axle ArticulationBody that rotates the driving wheel.
        /// </summary>
        public ArticulationBody SteerAxle { get => m_SteerAxle; set => m_SteerAxle = value; }
        /// <summary>
        /// The driving wheel ArticulationBody that moves the robot forward.
        /// </summary>
        public ArticulationBody DrivingWheel { get => m_DrivingWheel; set => m_DrivingWheel = value; }

        /// <summary>
        /// The pairs of forward wheels that act as caster wheels.
        /// </summary>
        public WheelPair[] FrontWheels { get => m_FrontWheels; set => m_FrontWheels = value; }
        /// <summary>
        /// Whether the controller is still executing the last control message or not.
        /// </summary>
        public float TargetLinearSpeed => m_LastMessage.speed;
        public float TargetSteeringAngle => m_LastMessage.steeringAngle;
        /// <summary>
        /// Last message received by this controller. If the <see cref="IsSleeping"/> property is false, it's still being acted upon.
        /// </summary>
        public ThreeWheelControlMessage LastMessage => m_LastMessage;

        /// <summary>
        /// Interpolated linear velocity that was last applied to the vehicle.
        /// </summary>
        public float CurrentTargetLinearVelocity => m_CurrentTargetVelocity;
        /// <summary>
        /// Interpolated linear acceleration that was last applied to the vehicle.
        /// </summary>
        public float CurrentTargetLinearAcceleration => m_CurrentTargetAcceleration;
        /// <summary>
        /// Interpolated steering angle that was last applied to the vehicle.
        /// </summary>
        public float CurrentTargetSteeringAngle => m_CurrentTargetSteeringAngle;

        /// <summary>
        /// The current steering axle rotation measured in degrees.
        /// </summary>
        public float CurrentWheelAngle => m_SteerAxle.xDrive.target;

        /// <summary>
        /// The wheel radius of the driving wheel measured in meters.
        /// </summary>
        public float WheelRadius => m_VehicleData.WheelRadius;
        /// <summary>
        /// The distance between the driving wheel and an average position of the front wheels.
        /// </summary>
        public float Wheelbase => m_VehicleData.Wheelbase;

        /// <summary>
        /// The forward direction in World Space
        /// </summary>
        public Vector3 GlobalForward => m_Root.transform.TransformDirection(m_LocalForwardDirection);
        /// <summary>
        /// Whether the controller is still executing the last control message or not.
        /// </summary>
        public bool IsSleeping => m_IsSleeping;
        /// <summary>
        /// Whether the vehicle is currently steering to the right.
        /// </summary>
        public bool IsSteeringToTheRight => m_CurrentTargetSteeringAngle < 0f;
        /// <summary>
        /// Whether the vehicle is steered with the driving wheel in the front or the back. 
        /// </summary>
        public bool IsFrontSteered => Wheelbase < 0f;
        /// <summary>
        /// Whether the vehicle is steering to the right and is not front steered. 
        /// </summary>
        public bool IsFlipped => IsSteeringToTheRight != IsFrontSteered;

        private void Start()
        {
            m_VehicleData.InitFromWheels(m_FrontWheels, m_DrivingWheel, GlobalForward);
        }

        /// <summary>
        /// This method updates the last message received in the controller.
        /// </summary>
        public void ConsumeMessage(ThreeWheelControlMessage message)
        {
            m_IsSleeping = false;
            m_LastMessage = message;
        }

        private void FixedUpdate()
        {
            if (m_IsSleeping)
                return;

            InterpolateTargets(Time.fixedDeltaTime);

            // Steering
            m_SteerAxle.SetXDriveTarget(m_CurrentTargetSteeringAngle * Mathf.Rad2Deg);

            // Driving
            var targetDPS = LinearVelocityToDegreesPerSecond(m_CurrentTargetVelocity, m_VehicleData.WheelRadius);
            m_DrivingWheel.SetXDriveTargetVelocity(targetDPS * m_VehicleData.VelocitySign);
        }

        private void InterpolateTargets(float dt)
        {
            m_CurrentTargetSteeringAngle = LinearlyInterpolateFloat(m_CurrentTargetSteeringAngle, m_LastMessage.steeringAngleVelocity, m_LastMessage.steeringAngle, dt);
            m_CurrentTargetAcceleration = LinearlyInterpolateFloat(m_CurrentTargetAcceleration, m_LastMessage.jerk, m_LastMessage.acceleration, dt);
            m_CurrentTargetVelocity = LinearlyInterpolateFloat(m_CurrentTargetVelocity, m_CurrentTargetAcceleration, m_LastMessage.speed, dt);

            // If both of the final parameters have been reached, fall asleep
            if (Mathf.Abs(m_CurrentTargetSteeringAngle - m_LastMessage.steeringAngle) <= MathUtilities.c_Epsilon && Mathf.Abs(m_CurrentTargetVelocity - m_LastMessage.speed) <= MathUtilities.c_Epsilon)
                m_IsSleeping = true;
        }

        private static float LinearlyInterpolateFloat(float current, float changePerSecond, float target, float dt)
        {
            if (current == target || changePerSecond == 0f)
                return target;

            var signBefore = Mathf.Sign(target - current);
            current += changePerSecond * dt * signBefore;
            var signAfter = Mathf.Sign(target - current);

            if (signAfter != signBefore) // If this is true, we overshot, can clamp
            {
                var maxStep = changePerSecond * dt;
                var distanceToTargetAfter = Mathf.Max(current, target) - Mathf.Min(current, target);
                Debug.Assert(distanceToTargetAfter <= maxStep + MathUtilities.c_Epsilon, $"Max step exeeded after overshooting! Max step: {maxStep} step:{distanceToTargetAfter}");
                current = target;
            }

            return current;
        }

        private static float LinearVelocityToDegreesPerSecond(float linearVelocity, float wheelRadius)
        {
            return linearVelocity / (Mathf.Deg2Rad * wheelRadius);
        }

        [Serializable]
        private struct VehicleData
        {
            public float Wheelbase;
            public float WheelRadius;
            public float VelocitySign;

            public void InitFromWheels(WheelPair[] frontPairs, ArticulationBody rearWheel, Vector3 forward)
            {
                Vector3 frontMidpoint = Vector3.zero;

                foreach (var pair in frontPairs)
                    frontMidpoint += (pair.leftWheel.transform.position + pair.rightWheel.transform.position) / 2;

                frontMidpoint /= frontPairs.Length;

                Wheelbase = (frontMidpoint - rearWheel.transform.position).magnitude;

                var rearToFront = frontMidpoint - rearWheel.transform.position;
                var sign = Mathf.Sign(Vector3.Dot(rearToFront, forward));
                VelocitySign = sign;
                Wheelbase *= sign;

                WheelRadius = GetWheelRadius(rearWheel);
            }

            private float GetWheelRadius(ArticulationBody ab)
            {
                var colliders = ab.GetComponentsInChildren<Collider>().Where(a => a.enabled);

                if (colliders.Count() == 0)
                {
                    Debug.LogError("Wheel ArticulationBody has no collider!");
                    return 0.5f;
                }

                var bounds = colliders.First().bounds;
                Assert.AreNotEqual(0f, bounds.extents.y, "Wheel radius was 0f!");
                return bounds.extents.y;
            }
        }
    }

}
