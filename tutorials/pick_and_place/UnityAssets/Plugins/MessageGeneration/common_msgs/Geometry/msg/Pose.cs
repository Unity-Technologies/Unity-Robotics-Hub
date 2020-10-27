using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Pose : Message
    {
        public const string RosMessageName = "geometry_msgs/Pose";

        //  A representation of pose in free space, composed of position and orientation. 
        public Point position { get; set; }
        public Quaternion orientation { get; set; }

        public Pose()
        {
            this.position = new Point();
            this.orientation = new Quaternion();
        }

        public Pose(Point position, Quaternion orientation)
        {
            this.position = position;
            this.orientation = orientation;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(position.SerializationStatements());
            listOfSerializations.AddRange(orientation.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.position.Deserialize(data, offset);
            offset = this.orientation.Deserialize(data, offset);

            return offset;
        }

    }
}
