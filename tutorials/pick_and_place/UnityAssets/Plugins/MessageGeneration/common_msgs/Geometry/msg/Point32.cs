using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Point32 : Message
    {
        public const string RosMessageName = "geometry_msgs/Point32";

        //  This contains the position of a point in free space(with 32 bits of precision).
        //  It is recommeded to use Point wherever possible instead of Point32.  
        //  
        //  This recommendation is to promote interoperability.  
        // 
        //  This message is designed to take up less space when sending
        //  lots of points at once, as in the case of a PointCloud.  
        public float x { get; set; }
        public float y { get; set; }
        public float z { get; set; }

        public Point32()
        {
            this.x = 0.0f;
            this.y = 0.0f;
            this.z = 0.0f;
        }

        public Point32(float x, float y, float z)
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
            this.x = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.y = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.z = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

    }
}
