using System;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// The Unity C# equivalent of a ROS message for mast control.
    /// </summary>
    [Serializable]
    public struct MastControlMessage : IControlMessage
    {
        public float mastVerticalExtension;         // Target vertical extension height
        public float mastVerticalSpeed;             // Speed at which the mast extends, 0 = as fast as possible
        public float forkLongitudinalExtension;     // Target forward extension
        public float forkLongitudinalSpeed;         // Speed at which the fork extends forward, 0 = as fast as possible
        public float forkTiltTarget;                // Target tilt angle of the fork
        public float forkTiltSpeed;                 // Speed at which the fork tilt, 0 = as fast as possible
        public float forkLateralExtension;          // Target lateral movement of the fork
        public float forkLateralSpeed;              // Speed at which the fork moves laterally, 0 = as fast as possible
    }
}
