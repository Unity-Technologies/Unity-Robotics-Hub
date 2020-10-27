using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class OrientationConstraint : Message
    {
        public const string RosMessageName = "moveit_msgs-master/OrientationConstraint";

        //  This message contains the definition of an orientation constraint.
        public Header header { get; set; }
        //  The desired orientation of the robot link specified as a quaternion
        public Quaternion orientation { get; set; }
        //  The robot link this constraint refers to
        public string link_name { get; set; }
        //  optional axis-angle error tolerances specified
        public double absolute_x_axis_tolerance { get; set; }
        public double absolute_y_axis_tolerance { get; set; }
        public double absolute_z_axis_tolerance { get; set; }
        //  A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
        public double weight { get; set; }

        public OrientationConstraint()
        {
            this.header = new Header();
            this.orientation = new Quaternion();
            this.link_name = "";
            this.absolute_x_axis_tolerance = 0.0;
            this.absolute_y_axis_tolerance = 0.0;
            this.absolute_z_axis_tolerance = 0.0;
            this.weight = 0.0;
        }

        public OrientationConstraint(Header header, Quaternion orientation, string link_name, double absolute_x_axis_tolerance, double absolute_y_axis_tolerance, double absolute_z_axis_tolerance, double weight)
        {
            this.header = header;
            this.orientation = orientation;
            this.link_name = link_name;
            this.absolute_x_axis_tolerance = absolute_x_axis_tolerance;
            this.absolute_y_axis_tolerance = absolute_y_axis_tolerance;
            this.absolute_z_axis_tolerance = absolute_z_axis_tolerance;
            this.weight = weight;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(orientation.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.link_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.absolute_x_axis_tolerance));
            listOfSerializations.Add(BitConverter.GetBytes(this.absolute_y_axis_tolerance));
            listOfSerializations.Add(BitConverter.GetBytes(this.absolute_z_axis_tolerance));
            listOfSerializations.Add(BitConverter.GetBytes(this.weight));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.orientation.Deserialize(data, offset);
            var link_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_name = DeserializeString(data, offset, link_nameStringBytesLength);
            offset += link_nameStringBytesLength;
            this.absolute_x_axis_tolerance = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.absolute_y_axis_tolerance = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.absolute_z_axis_tolerance = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.weight = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
