using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.NiryoOne;

namespace RosMessageTypes.NiryoOne
{
    public class Trajectory : Message
    {
        public const string RosMessageName = "niryo_one_msgs/Trajectory";

        public int id { get; set; }
        public string name { get; set; }
        public string description { get; set; }
        public TrajectoryPlan trajectory_plan { get; set; }

        public Trajectory()
        {
            this.id = 0;
            this.name = "";
            this.description = "";
            this.trajectory_plan = new TrajectoryPlan();
        }

        public Trajectory(int id, string name, string description, TrajectoryPlan trajectory_plan)
        {
            this.id = id;
            this.name = name;
            this.description = description;
            this.trajectory_plan = trajectory_plan;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.id));
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.Add(SerializeString(this.description));
            listOfSerializations.AddRange(trajectory_plan.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.id = BitConverter.ToInt32(data, offset);
            offset += 4;
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            var descriptionStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.description = DeserializeString(data, offset, descriptionStringBytesLength);
            offset += descriptionStringBytesLength;
            offset = this.trajectory_plan.Deserialize(data, offset);

            return offset;
        }

    }
}
