using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class JointConstraint : Message
    {
        public const string RosMessageName = "moveit_msgs-master/JointConstraint";

        //  Constrain the position of a joint to be within a certain bound
        public string joint_name { get; set; }
        //  the bound to be achieved is [position - tolerance_below, position + tolerance_above]
        public double position { get; set; }
        public double tolerance_above { get; set; }
        public double tolerance_below { get; set; }
        //  A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
        public double weight { get; set; }

        public JointConstraint()
        {
            this.joint_name = "";
            this.position = 0.0;
            this.tolerance_above = 0.0;
            this.tolerance_below = 0.0;
            this.weight = 0.0;
        }

        public JointConstraint(string joint_name, double position, double tolerance_above, double tolerance_below, double weight)
        {
            this.joint_name = joint_name;
            this.position = position;
            this.tolerance_above = tolerance_above;
            this.tolerance_below = tolerance_below;
            this.weight = weight;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.joint_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.position));
            listOfSerializations.Add(BitConverter.GetBytes(this.tolerance_above));
            listOfSerializations.Add(BitConverter.GetBytes(this.tolerance_below));
            listOfSerializations.Add(BitConverter.GetBytes(this.weight));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var joint_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.joint_name = DeserializeString(data, offset, joint_nameStringBytesLength);
            offset += joint_nameStringBytesLength;
            this.position = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.tolerance_above = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.tolerance_below = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.weight = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
