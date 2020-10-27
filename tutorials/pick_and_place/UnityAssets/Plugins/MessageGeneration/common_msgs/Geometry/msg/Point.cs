using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Point : Message
    {
        public const string RosMessageName = "geometry_msgs/Point";

        //  This contains the position of a point in free space
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public Point()
        {
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
        }

        public Point(double x, double y, double z)
        {
            this.x = x;
            this.y = y;
            this.z = z;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.x));
            listOfSerializations.Add(BitConverter.GetBytes(this.y));
            listOfSerializations.Add(BitConverter.GetBytes(this.z));

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

            return offset;
        }

    }
}
