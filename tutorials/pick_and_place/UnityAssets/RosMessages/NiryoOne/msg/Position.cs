using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.NiryoOne;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.NiryoOne
{
    public class Position : Message
    {
        public const string RosMessageName = "niryo_one_msgs/Position";

        public string name { get; set; }
        public double[] joints { get; set; }
        public RPY rpy { get; set; }
        public Point point { get; set; }
        public Quaternion quaternion { get; set; }

        public Position()
        {
            this.name = "";
            this.joints = new double[0];
            this.rpy = new RPY();
            this.point = new Point();
            this.quaternion = new Quaternion();
        }

        public Position(string name, double[] joints, RPY rpy, Point point, Quaternion quaternion)
        {
            this.name = name;
            this.joints = joints;
            this.rpy = rpy;
            this.point = point;
            this.quaternion = quaternion;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            
            listOfSerializations.Add(BitConverter.GetBytes(joints.Length));
            foreach(var entry in joints)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            listOfSerializations.AddRange(rpy.SerializationStatements());
            listOfSerializations.AddRange(point.SerializationStatements());
            listOfSerializations.AddRange(quaternion.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            
            var jointsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.joints= new double[jointsArrayLength];
            for(var i =0; i <jointsArrayLength; i++)
            {
                this.joints[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            offset = this.rpy.Deserialize(data, offset);
            offset = this.point.Deserialize(data, offset);
            offset = this.quaternion.Deserialize(data, offset);

            return offset;
        }

    }
}
