using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Moveit;

namespace RosMessageTypes.NiryoOne
{
    public class TrajectoryPlan : Message
    {
        public const string RosMessageName = "niryo_one_msgs/TrajectoryPlan";

        public Moveit.RobotState trajectory_start { get; set; }
        public string group_name { get; set; }
        public RosMessageTypes.Moveit.RobotTrajectory trajectory { get; set; }

        public TrajectoryPlan()
        {
            this.trajectory_start = new RosMessageTypes.Moveit.RobotState();
            this.group_name = "";
            this.trajectory = new RosMessageTypes.Moveit.RobotTrajectory();
        }

        public TrajectoryPlan(Moveit.RobotState trajectory_start, string group_name, RobotTrajectory trajectory)
        {
            this.trajectory_start = trajectory_start;
            this.group_name = group_name;
            this.trajectory = trajectory;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(trajectory_start.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.group_name));
            listOfSerializations.AddRange(trajectory.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.trajectory_start.Deserialize(data, offset);
            var group_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.group_name = DeserializeString(data, offset, group_nameStringBytesLength);
            offset += group_nameStringBytesLength;
            offset = this.trajectory.Deserialize(data, offset);

            return offset;
        }

    }
}
