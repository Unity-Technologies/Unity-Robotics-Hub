using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Vector3 : Message
    {
        public const string RosMessageName = "geometry_msgs/Vector3";

        //  This represents a vector in free space. 
        //  It is only meant to represent a direction. Therefore, it does not
        //  make sense to apply a translation to it (e.g., when applying a 
        //  generic rigid transformation to a Vector3, tf2 will only apply the
        //  rotation). If you want your data to be translatable too, use the
        //  geometry_msgs/Point message instead.
        public double x { get; set; }
        public double y { get; set; }
        public double z { get; set; }

        public Vector3()
        {
            this.x = 0.0;
            this.y = 0.0;
            this.z = 0.0;
        }

        public Vector3(double x, double y, double z)
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
