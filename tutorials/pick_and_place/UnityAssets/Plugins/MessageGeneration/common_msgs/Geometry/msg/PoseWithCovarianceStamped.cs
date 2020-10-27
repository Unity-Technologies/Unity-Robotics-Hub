using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class PoseWithCovarianceStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/PoseWithCovarianceStamped";

        //  This expresses an estimated pose with a reference coordinate frame and timestamp
        public Header header { get; set; }
        public PoseWithCovariance pose { get; set; }

        public PoseWithCovarianceStamped()
        {
            this.header = new Header();
            this.pose = new PoseWithCovariance();
        }

        public PoseWithCovarianceStamped(Header header, PoseWithCovariance pose)
        {
            this.header = header;
            this.pose = pose;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(pose.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.pose.Deserialize(data, offset);

            return offset;
        }

    }
}
