using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class ShiftPose : Message
    {
        public const string RosMessageName = "niryo_one_msgs/ShiftPose";

        public int axis_number { get; set; }
        public double value { get; set; }

        public ShiftPose()
        {
            this.axis_number = 0;
            this.value = 0.0;
        }

        public ShiftPose(int axis_number, double value)
        {
            this.axis_number = axis_number;
            this.value = value;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.axis_number));
            listOfSerializations.Add(BitConverter.GetBytes(this.value));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.axis_number = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.value = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
