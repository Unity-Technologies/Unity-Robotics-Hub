using System.Collections;
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// A helper class for fetching the various interfaces with specific message types.
    /// </summary>
    public static class InterfaceFetchingExtensions
    {
        /// <summary>
        /// Return the controller interface for the provided type of message attahced to this object.
        /// </summary>
        public static IRobotController<T> FetchControllerWithMessageType<T>(this MonoBehaviour script) where T : IControlMessage
        {
            var components = script.GetComponents<MonoBehaviour>();
            if (components == null || components.Length == 0)
                Debug.LogError("No MonoBehaviours found on the GameObject!");

            var validComponents = components.OfType<IRobotController<T>>();

            if (validComponents == null || validComponents.Count() != 1)
                Debug.LogError($"Couldn't find the {typeof(IRobotController<T>).Name} controller!");

            return validComponents.First();
        }
    }
}
