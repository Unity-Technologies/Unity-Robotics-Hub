using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class JointLimits : Message
    {
        public const string RosMessageName = "moveit_msgs-master/JointLimits";

        //  This message contains information about limits of a particular joint (or control dimension)
        public string joint_name { get; set; }
        //  true if the joint has position limits
        public bool has_position_limits { get; set; }
        //  min and max position limits
        public double min_position { get; set; }
        public double max_position { get; set; }
        //  true if joint has velocity limits
        public bool has_velocity_limits { get; set; }
        //  max velocity limit
        public double max_velocity { get; set; }
        //  min_velocity is assumed to be -max_velocity
        //  true if joint has acceleration limits
        public bool has_acceleration_limits { get; set; }
        //  max acceleration limit
        public double max_acceleration { get; set; }
        //  min_acceleration is assumed to be -max_acceleration

        public JointLimits()
        {
            this.joint_name = "";
            this.has_position_limits = false;
            this.min_position = 0.0;
            this.max_position = 0.0;
            this.has_velocity_limits = false;
            this.max_velocity = 0.0;
            this.has_acceleration_limits = false;
            this.max_acceleration = 0.0;
        }

        public JointLimits(string joint_name, bool has_position_limits, double min_position, double max_position, bool has_velocity_limits, double max_velocity, bool has_acceleration_limits, double max_acceleration)
        {
            this.joint_name = joint_name;
            this.has_position_limits = has_position_limits;
            this.min_position = min_position;
            this.max_position = max_position;
            this.has_velocity_limits = has_velocity_limits;
            this.max_velocity = max_velocity;
            this.has_acceleration_limits = has_acceleration_limits;
            this.max_acceleration = max_acceleration;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.joint_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.has_position_limits));
            listOfSerializations.Add(BitConverter.GetBytes(this.min_position));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_position));
            listOfSerializations.Add(BitConverter.GetBytes(this.has_velocity_limits));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_velocity));
            listOfSerializations.Add(BitConverter.GetBytes(this.has_acceleration_limits));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_acceleration));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var joint_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.joint_name = DeserializeString(data, offset, joint_nameStringBytesLength);
            offset += joint_nameStringBytesLength;
            this.has_position_limits = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.min_position = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.max_position = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.has_velocity_limits = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.max_velocity = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.has_acceleration_limits = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.max_acceleration = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
