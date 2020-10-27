using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Geometry
{
    public class PoseArray : Message
    {
        public const string RosMessageName = "geometry_msgs/PoseArray";

        //  An array of poses with a header for global reference.
        public Header header { get; set; }
        public Pose[] poses { get; set; }

        public PoseArray()
        {
            this.header = new Header();
            this.poses = new Pose[0];
        }

        public PoseArray(Header header, Pose[] poses)
        {
            this.header = header;
            this.poses = poses;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(poses.Length));
            foreach(var entry in poses)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.poses= new Pose[posesArrayLength];
            for(var i =0; i <posesArrayLength; i++)
            {
                this.poses[i] = new Pose();
                offset = this.poses[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
