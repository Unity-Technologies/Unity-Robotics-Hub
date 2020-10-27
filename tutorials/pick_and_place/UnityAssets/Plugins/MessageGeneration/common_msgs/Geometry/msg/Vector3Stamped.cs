using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class Vector3Stamped : Message
    {
        public const string RosMessageName = "geometry_msgs/Vector3Stamped";

        //  This represents a Vector3 with reference coordinate frame and timestamp
        public Header header { get; set; }
        public Vector3 vector { get; set; }

        public Vector3Stamped()
        {
            this.header = new Header();
            this.vector = new Vector3();
        }

        public Vector3Stamped(Header header, Vector3 vector)
        {
            this.header = header;
            this.vector = vector;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(vector.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.vector.Deserialize(data, offset);

            return offset;
        }

    }
}
