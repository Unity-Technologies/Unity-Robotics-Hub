using System;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// The Unity C# equivalent of the ROS Twist message.
    /// </summary>
    [Serializable]
    public struct DifferentialDriveControlMessage : IControlMessage
    {
        // This message operates in the Unity Left-Hand coordinate system
        public Vector3 linear;         // desired forward velocity along forward vector (m/s). Only z is used
        public Vector3 angular;        // desired angular velocity around up axis (rad/s). Only y is used

        public override string ToString()
        {
            return $"(Linear.x: {linear.x}, Linear.y: {linear.y}, Linear.z: {linear.z}, Angular.x: {angular.x}, Angular.y: {angular.y}, Angular.z: {angular.z})";
        }
    }
}
