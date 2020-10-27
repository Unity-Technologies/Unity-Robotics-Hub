using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Nav
{
    public class Path : Message
    {
        public const string RosMessageName = "nav_msgs/Path";

        // An array of poses that represents a Path for a robot to follow
        public Header header { get; set; }
        public PoseStamped[] poses { get; set; }

        public Path()
        {
            this.header = new Header();
            this.poses = new PoseStamped[0];
        }

        public Path(Header header, PoseStamped[] poses)
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
            this.poses= new PoseStamped[posesArrayLength];
            for(var i =0; i <posesArrayLength; i++)
            {
                this.poses[i] = new PoseStamped();
                offset = this.poses[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
