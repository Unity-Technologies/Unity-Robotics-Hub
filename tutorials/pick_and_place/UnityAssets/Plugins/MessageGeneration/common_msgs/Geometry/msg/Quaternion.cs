using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Quaternion : Message
    {
        public const string RosMessageName = "geometry_msgs/Quaternion";

        //  This represents an orientation in free space in quaternion form.
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }
        public double w { get; set; }

        public Quaternion()
        {
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
            this.w = 0.0;
        }

        public Quaternion(double x, double y, double z, double w)
        {
            this.x = x;
            this.y = y;
            this.z = z;
            this.w = w;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.x));
            listOfSerializations.Add(BitConverter.GetBytes(this.y));
            listOfSerializations.Add(BitConverter.GetBytes(this.z));
            listOfSerializations.Add(BitConverter.GetBytes(this.w));

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
            this.w = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
