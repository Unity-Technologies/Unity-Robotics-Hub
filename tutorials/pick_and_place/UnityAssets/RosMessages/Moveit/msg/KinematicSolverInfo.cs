using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Moveit;

namespace RosMessageTypes.Moveit
{
    public class KinematicSolverInfo : Message
    {
        public const string RosMessageName = "moveit_msgs-master/KinematicSolverInfo";

        //  A list of joints in the kinematic tree
        public string[] joint_names { get; set; }
        //  A list of joint limits corresponding to the joint names
        public JointLimits[] limits { get; set; }
        //  A list of links that the kinematics node provides solutions for
        public string[] link_names { get; set; }

        public KinematicSolverInfo()
        {
            this.joint_names = new string[0];
            this.limits = new JointLimits[0];
            this.link_names = new string[0];
        }

        public KinematicSolverInfo(string[] joint_names, JointLimits[] limits, string[] link_names)
        {
            this.joint_names = joint_names;
            this.limits = limits;
            this.link_names = link_names;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(joint_names.Length));
            foreach(var entry in joint_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(limits.Length));
            foreach(var entry in limits)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(link_names.Length));
            foreach(var entry in link_names)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var joint_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.joint_names= new string[joint_namesArrayLength];
            for(var i =0; i <joint_namesArrayLength; i++)
            {
                var joint_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.joint_names[i] = DeserializeString(data, offset, joint_namesStringBytesLength);
                offset += joint_namesStringBytesLength;
            }
            
            var limitsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.limits= new JointLimits[limitsArrayLength];
            for(var i =0; i <limitsArrayLength; i++)
            {
                this.limits[i] = new JointLimits();
                offset = this.limits[i].Deserialize(data, offset);
            }
            
            var link_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_names= new string[link_namesArrayLength];
            for(var i =0; i <link_namesArrayLength; i++)
            {
                var link_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.link_names[i] = DeserializeString(data, offset, link_namesStringBytesLength);
                offset += link_namesStringBytesLength;
            }

            return offset;
        }

    }
}
