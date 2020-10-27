using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Pose2D : Message
    {
        public const string RosMessageName = "geometry_msgs/Pose2D";

        //  Deprecated
        //  Please use the full 3D pose.
        //  In general our recommendation is to use a full 3D representation of everything and for 2D specific applications make the appropriate projections into the plane for their calculations but optimally will preserve the 3D information during processing.
        //  If we have parallel copies of 2D datatypes every UI and other pipeline will end up needing to have dual interfaces to plot everything. And you will end up with not being able to use 3D tools for 2D use cases even if they're completely valid, as you'd have to reimplement it with different inputs and outputs. It's not particularly hard to plot the 2D pose or compute the yaw error for the Pose message and there are already tools and libraries that can do this for you.
        //  This expresses a position and orientation on a 2D manifold.
        public double x { get; set; }
        public double y { get; set; }
        public double theta { get; set; }

        public Pose2D()
        {
            this.x = 0.0;
            this.y = 0.0;
            this.theta = 0.0;
        }

        public Pose2D(double x, double y, double theta)
        {
            this.x = x;
            this.y = y;
            this.theta = theta;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.x));
            listOfSerializations.Add(BitConverter.GetBytes(this.y));
            listOfSerializations.Add(BitConverter.GetBytes(this.theta));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.x = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.y = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.theta = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
