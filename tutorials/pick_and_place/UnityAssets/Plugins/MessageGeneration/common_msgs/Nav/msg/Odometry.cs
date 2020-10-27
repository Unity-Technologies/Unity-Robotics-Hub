using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Nav
{
    public class Odometry : Message
    {
        public const string RosMessageName = "nav_msgs/Odometry";

        //  This represents an estimate of a position and velocity in free space.  
        //  The pose in this message should be specified in the coordinate frame given by header.frame_id.
        //  The twist in this message should be specified in the coordinate frame given by the child_frame_id
        public Header header { get; set; }
        public string child_frame_id { get; set; }
        public PoseWithCovariance pose { get; set; }
        public TwistWithCovariance twist { get; set; }

        public Odometry()
        {
            this.header = new Header();
            this.child_frame_id = "";
            this.pose = new PoseWithCovariance();
            this.twist = new TwistWithCovariance();
        }

        public Odometry(Header header, string child_frame_id, PoseWithCovariance pose, TwistWithCovariance twist)
        {
            this.header = header;
            this.child_frame_id = child_frame_id;
            this.pose = pose;
            this.twist = twist;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.child_frame_id));
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.AddRange(twist.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            var child_frame_idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.child_frame_id = DeserializeString(data, offset, child_frame_idStringBytesLength);
            offset += child_frame_idStringBytesLength;
            offset = this.pose.Deserialize(data, offset);
            offset = this.twist.Deserialize(data, offset);

            return offset;
        }

    }
}
