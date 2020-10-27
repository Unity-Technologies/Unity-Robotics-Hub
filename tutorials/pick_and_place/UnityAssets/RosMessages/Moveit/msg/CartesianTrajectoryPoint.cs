using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Moveit
{
    public class CartesianTrajectoryPoint : Message
    {
        public const string RosMessageName = "moveit_msgs-master/CartesianTrajectoryPoint";

        //  The definition of a cartesian point in a trajectory. Defines the cartesian state of the point and it's time,
        //  following the pattern of the JointTrajectory message
        public CartesianPoint point { get; set; }
        public Duration time_from_start { get; set; }

        public CartesianTrajectoryPoint()
        {
            this.point = new CartesianPoint();
            this.time_from_start = new Duration();
        }

        public CartesianTrajectoryPoint(CartesianPoint point, Duration time_from_start)
        {
            this.point = point;
            this.time_from_start = time_from_start;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(point.SerializationStatements());
            listOfSerializations.AddRange(time_from_start.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.point.Deserialize(data, offset);
            offset = this.time_from_start.Deserialize(data, offset);

            return offset;
        }

    }
}
