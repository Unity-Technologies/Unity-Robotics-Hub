using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Trajectory
{
    public class JointTrajectory : Message
    {
        public const string RosMessageName = "trajectory_msgs/JointTrajectory";

        public Header header { get; set; }
        public string[] joint_names { get; set; }
        public JointTrajectoryPoint[] points { get; set; }

        public JointTrajectory()
        {
            this.header = new Header();
            this.joint_names = new string[0];
            this.points = new JointTrajectoryPoint[0];
        }

        public JointTrajectory(Header header, string[] joint_names, JointTrajectoryPoint[] points)
        {
            this.header = header;
            this.joint_names = joint_names;
            this.points = points;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(joint_names.Length));
            foreach(var entry in joint_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(points.Length));
            foreach(var entry in points)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
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
            
            var pointsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.points= new JointTrajectoryPoint[pointsArrayLength];
            for(var i =0; i <pointsArrayLength; i++)
            {
                this.points[i] = new JointTrajectoryPoint();
                offset = this.points[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
