using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class AccelWithCovarianceStamped : Message
    {
        public const string RosMessageName = "geometry_msgs/AccelWithCovarianceStamped";

        //  This represents an estimated accel with reference coordinate frame and timestamp.
        public Header header { get; set; }
        public AccelWithCovariance accel { get; set; }

        public AccelWithCovarianceStamped()
        {
            this.header = new Header();
            this.accel = new AccelWithCovariance();
        }

        public AccelWithCovarianceStamped(Header header, AccelWithCovariance accel)
        {
            this.header = header;
            this.accel = accel;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(accel.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.accel.Deserialize(data, offset);

            return offset;
        }

    }
}
