using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class QuaternionStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/QuaternionStamped";

        //  This represents an orientation with reference coordinate frame and timestamp.
        public Header header { get; set; }
        public Quaternion quaternion { get; set; }

        public QuaternionStamped()
        {
            this.header = new Header();
            this.quaternion = new Quaternion();
        }

        public QuaternionStamped(Header header, Quaternion quaternion)
        {
            this.header = header;
            this.quaternion = quaternion;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(quaternion.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.quaternion.Deserialize(data, offset);

            return offset;
        }

    }
}
