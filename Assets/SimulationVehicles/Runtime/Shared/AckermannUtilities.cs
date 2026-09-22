using System.Collections;
using System.Collections.Generic;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// A helper class that contains methods that calculate certain parameters
    /// related to the Ackermann drive.
    /// </summary>
    public static class AckermannUtilities
    {
        //        (wheelbase positive) | (wheelbase negative)
        //               │ Wheel ahead │ Wheel behind │
        // --------------┼-------------┼--------------┼--------------
        // Turning left  │    negative │     positive │ (gamma negative)
        // --------------┼-------------┼--------------┼--------------
        // Turning right │    positive │     negative │ (gamma positive)

        /// <summary>
        /// Returns the wheel angles in radians that the inner and outer wheels
        /// should have, based on the imaginary middle wheel (gamma) rotation.
        /// </summary>
        public static (/*Left*/float, /*Right*/float) GetWheelAnglesRad(float baseRadius, float gamma, float wheelbase, float trackWidth)
        {
            // Function is meaningless with gamma ≈ 0f
            // The function is undefined with wheelbase = 0f
            if (Mathf.Abs(gamma) <= MathUtilities.c_Epsilon || Mathf.Abs(wheelbase) == 0f)
                return (0f, 0f);

            float alpha;
            float beta;

            // The wheel is "ahead" of the reference point
            if (wheelbase > 0f)
            {
                alpha = Mathf.PI / 2f - Mathf.Atan((baseRadius - trackWidth / 2f) / wheelbase);
                beta = Mathf.PI / 2f - Mathf.Atan((baseRadius + trackWidth / 2f) / wheelbase);
            }
            // The wheel is "behind" the reference point
            else
            {
                alpha = Mathf.Atan((baseRadius - trackWidth / 2f) / wheelbase) + Mathf.PI / 2f;
                beta = Mathf.Atan((baseRadius + trackWidth / 2f) / wheelbase) + Mathf.PI / 2f;
            }

            alpha = Mathf.Clamp(alpha, -Mathf.PI / 2f, Mathf.PI / 2f);
            beta = Mathf.Clamp(beta, -Mathf.PI / 2f, Mathf.PI / 2f);

            bool flipSign = (gamma < 0f) ^ (wheelbase < 0f);
            alpha = flipSign ? -alpha : alpha;
            beta = flipSign ? -beta : beta;

            return gamma > 0f ? (beta, alpha) : (alpha, beta);
        }

        /// <summary>
        /// Get the radius of the circle that the vehicle is rotating around from
        /// the imaginary middle wheel rotation (gamma).
        /// </summary>
        public static float GetBaseRadiusFromGamma(float gamma, float distanceBetweenSteeredAndNonSteered) // Gamma in rad
        {
            return Mathf.Tan(Mathf.PI / 2f - Mathf.Abs(gamma)) * distanceBetweenSteeredAndNonSteered;
        }
        /// <summary>
        /// Returns a linear speed value generated from degrees per second and the wheel radius provided.
        /// </summary>
        public static float DegreesPerSecondToLinearSpeed(float degreesPerSecond, float wheelRadius)
        {
            return (degreesPerSecond * Mathf.Deg2Rad) * wheelRadius;
        }

        /// <summary>
        /// Returns degrees per second generated from the linear speed value and the wheel radius provided.
        /// </summary>
        public static float LinearSpeedToDegreesPerSecond(float linearVelocity, float wheelRadius)
        {
            return linearVelocity / (Mathf.Deg2Rad * wheelRadius);
        }
    }
}
