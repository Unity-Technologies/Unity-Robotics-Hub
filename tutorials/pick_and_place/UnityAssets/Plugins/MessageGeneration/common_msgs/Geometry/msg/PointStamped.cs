using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class PointStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/PointStamped";

        //  This represents a Point with reference coordinate frame and timestamp
        public Header header { get; set; }
        public Point point { get; set; }

        public PointStamped()
        {
            this.header = new Header();
            this.point = new Point();
        }

        public PointStamped(Header header, Point point)
        {
            this.header = header;
            this.point = point;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(point.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.point.Deserialize(data, offset);

            return offset;
        }

    }
}
