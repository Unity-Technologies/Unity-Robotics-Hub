using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class LinkScale : Message
    {
        public const string RosMessageName = "moveit_msgs-master/LinkScale";

        // name for the link
        public string link_name { get; set; }
        //  scaling to apply to the link
        public double scale { get; set; }

        public LinkScale()
        {
            this.link_name = "";
            this.scale = 0.0;
        }

        public LinkScale(string link_name, double scale)
        {
            this.link_name = link_name;
            this.scale = scale;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.link_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.scale));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var link_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_name = DeserializeString(data, offset, link_nameStringBytesLength);
            offset += link_nameStringBytesLength;
            this.scale = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
