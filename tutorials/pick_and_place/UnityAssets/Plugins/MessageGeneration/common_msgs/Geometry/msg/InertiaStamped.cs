using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class InertiaStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/InertiaStamped";

        public Header header { get; set; }
        public Inertia inertia { get; set; }

        public InertiaStamped()
        {
            this.header = new Header();
            this.inertia = new Inertia();
        }

        public InertiaStamped(Header header, Inertia inertia)
        {
            this.header = header;
            this.inertia = inertia;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(inertia.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.inertia.Deserialize(data, offset);

            return offset;
        }

    }
}
