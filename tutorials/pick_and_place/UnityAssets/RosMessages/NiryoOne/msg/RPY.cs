using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class RPY : Message
    {
        public const string RosMessageName = "niryo_one_msgs/RPY";

        //  roll, pitch and yaw
        public double roll { get; set; }
        public double pitch { get; set; }
        public double yaw { get; set; }

        public RPY()
        {
            this.roll = 0.0;
            this.pitch = 0.0;
            this.yaw = 0.0;
        }

        public RPY(double roll, double pitch, double yaw)
        {
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.roll));
            listOfSerializations.Add(BitConverter.GetBytes(this.pitch));
            listOfSerializations.Add(BitConverter.GetBytes(this.yaw));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.roll = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.pitch = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.yaw = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
