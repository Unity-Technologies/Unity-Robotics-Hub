using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class TwistWithCovarianceStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/TwistWithCovarianceStamped";

        //  This represents an estimated twist with reference coordinate frame and timestamp.
        public Header header { get; set; }
        public TwistWithCovariance twist { get; set; }

        public TwistWithCovarianceStamped()
        {
            this.header = new Header();
            this.twist = new TwistWithCovariance();
        }

        public TwistWithCovarianceStamped(Header header, TwistWithCovariance twist)
        {
            this.header = header;
            this.twist = twist;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(twist.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.twist.Deserialize(data, offset);

            return offset;
        }

    }
}
