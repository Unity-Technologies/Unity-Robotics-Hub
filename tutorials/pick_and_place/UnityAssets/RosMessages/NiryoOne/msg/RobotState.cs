using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoOne;

namespace RosMessageTypes.NiryoOne
{
    public class RobotState : Message
    {
        public const string RosMessageName = "niryo_one_msgs/RobotState";

        public Point position { get; set; }
        public RPY rpy { get; set; }

        public RobotState()
        {
            this.position = new Point();
            this.rpy = new RPY();
        }

        public RobotState(Point position, RPY rpy)
        {
            this.position = position;
            this.rpy = rpy;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(position.SerializationStatements());
            listOfSerializations.AddRange(rpy.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.position.Deserialize(data, offset);
            offset = this.rpy.Deserialize(data, offset);

            return offset;
        }

    }
}
