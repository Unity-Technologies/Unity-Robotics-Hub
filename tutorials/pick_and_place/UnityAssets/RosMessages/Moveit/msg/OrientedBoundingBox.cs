using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class OrientedBoundingBox : Message
    {
        public const string RosMessageName = "moveit_msgs-master/OrientedBoundingBox";

        //  the pose of the box
        public Pose pose { get; set; }
        //  the extents of the box, assuming the center is at the origin
        public Point32 extents { get; set; }

        public OrientedBoundingBox()
        {
            this.pose = new Pose();
            this.extents = new Point32();
        }

        public OrientedBoundingBox(Pose pose, Point32 extents)
        {
            this.pose = pose;
            this.extents = extents;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.AddRange(extents.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.pose.Deserialize(data, offset);
            offset = this.extents.Deserialize(data, offset);

            return offset;
        }

    }
}
