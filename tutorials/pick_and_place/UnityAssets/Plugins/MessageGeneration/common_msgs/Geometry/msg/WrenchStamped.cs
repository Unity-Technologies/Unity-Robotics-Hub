using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class WrenchStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/WrenchStamped";

        //  A wrench with reference coordinate frame and timestamp
        public Header header { get; set; }
        public Wrench wrench { get; set; }

        public WrenchStamped()
        {
            this.header = new Header();
            this.wrench = new Wrench();
        }

        public WrenchStamped(Header header, Wrench wrench)
        {
            this.header = header;
            this.wrench = wrench;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(wrench.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.wrench.Deserialize(data, offset);

            return offset;
        }

    }
}
