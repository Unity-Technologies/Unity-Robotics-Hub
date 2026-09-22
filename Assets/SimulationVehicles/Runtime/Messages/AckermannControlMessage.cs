using System;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// The Unity C# equivalent of the ROS Ackermann message.
    /// </summary>
    [Serializable]
    public struct AckermannControlMessage : IControlMessage
    {
        public float steeringAngle;         // desired virtual angle (radians)
        public float steeringAngleVelocity; // desired rate of change (radians/s)
        public float speed;                 // desired forward speed (m/s)
        public float acceleration;          // desired acceleration (m/s^2)
        public float jerk;                  // desired jerk (m/s^3)

        public override string ToString()
        {
            return $"(SteeringAngle: {steeringAngle}, SteeringAngleVelocity: {steeringAngleVelocity}, Speed: {speed}, Acceleration: {acceleration}, Jerk: {jerk})";
        }
    }
}
