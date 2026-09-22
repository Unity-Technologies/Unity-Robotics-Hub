
namespace Unity.Simulation.VehicleControllers
{
    /// <summary>
    /// This interface is implemented by all the controllers.
    /// </summary>
    public interface IRobotController<ControlMessage> where ControlMessage : IControlMessage
    {
        /// <summary>
        /// This method is used to take the incoming message and convert it to
        /// ArticulationBody drive values
        /// </summary>
        public void ConsumeMessage(ControlMessage message);
        /// <summary>
        /// Last message received by the controller.
        /// </summary>
        public ControlMessage LastMessage { get; }
    }
}
