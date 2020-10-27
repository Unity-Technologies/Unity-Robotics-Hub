using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class LinkPadding : Message
    {
        public const string RosMessageName = "moveit_msgs-master/LinkPadding";

        // name for the link
        public string link_name { get; set; }
        //  padding to apply to the link
        public double padding { get; set; }

        public LinkPadding()
        {
            this.link_name = "";
            this.padding = 0.0;
        }

        public LinkPadding(string link_name, double padding)
        {
            this.link_name = link_name;
            this.padding = padding;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.link_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.padding));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var link_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_name = DeserializeString(data, offset, link_nameStringBytesLength);
            offset += link_nameStringBytesLength;
            this.padding = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
