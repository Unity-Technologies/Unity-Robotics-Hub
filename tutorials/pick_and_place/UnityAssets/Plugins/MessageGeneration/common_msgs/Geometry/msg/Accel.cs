using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Accel : Message
    {
        public const string RosMessageName = "geometry_msgs/Accel";

        //  This expresses acceleration in free space broken into its linear and angular parts.
        public Vector3 linear { get; set; }
        public Vector3 angular { get; set; }

        public Accel()
        {
            this.linear = new Vector3();
            this.angular = new Vector3();
        }

        public Accel(Vector3 linear, Vector3 angular)
        {
            this.linear = linear;
            this.angular = angular;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(linear.SerializationStatements());
            listOfSerializations.AddRange(angular.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.linear.Deserialize(data, offset);
            offset = this.angular.Deserialize(data, offset);

            return offset;
        }

    }
}
