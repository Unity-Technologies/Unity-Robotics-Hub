using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    public static class MathUtilities
    {
        /// <summary>
        /// A generic Epsilon value for use in float comparisons.
        /// </summary>
        public const float c_Epsilon = 0.0001f;

        /// <summary>
        /// Interpolate a float value linearly and check against an Epsilon if the value has been reached.
        /// </summary>
        public static float LinearlyInterpolateFloat(float current, float changePerSecond, float target, float dt, float epsilon = 0.0001f)
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
                Debug.Assert(distanceToTargetAfter <= maxStep + epsilon, $"Max step exeeded after overshooting! Max step: {maxStep} step:{distanceToTargetAfter}");
                current = target;
            }

            return current;
        }

        /// <summary>
        /// Get a semi-random color based on the hash value.
        /// </summary>
        public static Color GetColorFromHash(int hash)
        {
            hash ^= hash >> 6;
            hash ^= hash << 13;
            hash ^= hash >> 14;
            hash *= 736338717;

            return new Color32(
                (byte)hash,
                (byte)(hash >> 8),
                (byte)(hash >> 16),
                255
            );
        }
    }
}
