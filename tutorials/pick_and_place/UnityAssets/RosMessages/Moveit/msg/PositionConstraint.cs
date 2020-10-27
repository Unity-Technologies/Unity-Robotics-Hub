using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class PositionConstraint : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PositionConstraint";

        //  This message contains the definition of a position constraint.
        public Header header { get; set; }
        //  The robot link this constraint refers to
        public string link_name { get; set; }
        //  The offset (in the link frame) for the target point on the link we are planning for
        public Vector3 target_point_offset { get; set; }
        //  The volume this constraint refers to 
        public BoundingVolume constraint_region { get; set; }
        //  A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
        public double weight { get; set; }

        public PositionConstraint()
        {
            this.header = new Header();
            this.link_name = "";
            this.target_point_offset = new Vector3();
            this.constraint_region = new BoundingVolume();
            this.weight = 0.0;
        }

        public PositionConstraint(Header header, string link_name, Vector3 target_point_offset, BoundingVolume constraint_region, double weight)
        {
            this.header = header;
            this.link_name = link_name;
            this.target_point_offset = target_point_offset;
            this.constraint_region = constraint_region;
            this.weight = weight;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.link_name));
            listOfSerializations.AddRange(target_point_offset.SerializationStatements());
            listOfSerializations.AddRange(constraint_region.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.weight));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            var link_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_name = DeserializeString(data, offset, link_nameStringBytesLength);
            offset += link_nameStringBytesLength;
            offset = this.target_point_offset.Deserialize(data, offset);
            offset = this.constraint_region.Deserialize(data, offset);
            this.weight = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
