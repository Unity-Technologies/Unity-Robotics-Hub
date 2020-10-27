using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Wrench : Message
    {
        public const string RosMessageName = "geometry_msgs/Wrench";

        //  This represents force in free space, separated into
        //  its linear and angular parts.
        public Vector3 force { get; set; }
        public Vector3 torque { get; set; }

        public Wrench()
        {
            this.force = new Vector3();
            this.torque = new Vector3();
        }

        public Wrench(Vector3 force, Vector3 torque)
        {
            this.force = force;
            this.torque = torque;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(force.SerializationStatements());
            listOfSerializations.AddRange(torque.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.force.Deserialize(data, offset);
            offset = this.torque.Deserialize(data, offset);

            return offset;
        }

    }
}
