using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class ObjectPose : Message
    {
        public const string RosMessageName = "niryo_one_msgs/ObjectPose";

        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double roll { get; set; }
        public double pitch { get; set; }
        public double yaw { get; set; }

        public ObjectPose()
        {
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
            this.roll = 0.0;
            this.pitch = 0.0;
            this.yaw = 0.0;
        }

        public ObjectPose(double x, double y, double z, double roll, double pitch, double yaw)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.roll = roll;
            this.pitch = pitch;
            this.yaw = yaw;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.x));
            listOfSerializations.Add(BitConverter.GetBytes(this.y));
            listOfSerializations.Add(BitConverter.GetBytes(this.z));
            listOfSerializations.Add(BitConverter.GetBytes(this.roll));
            listOfSerializations.Add(BitConverter.GetBytes(this.pitch));
            listOfSerializations.Add(BitConverter.GetBytes(this.yaw));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.x = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.y = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.z = BitConverter.ToDouble(data, offset);
            offset += 8;
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
