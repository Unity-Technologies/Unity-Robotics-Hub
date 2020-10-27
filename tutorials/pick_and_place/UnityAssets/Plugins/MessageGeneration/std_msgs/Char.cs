using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Char : Message
    {
        public const string RosMessageName = "std_msgs/Char";

        public byte data { get; set; }

        public Char()
        {
            this.data = 0;
        }

        public Char(byte data)
        {
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.data));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.data = data[offset];;
            offset += 1;

            return offset;
        }

    }
}
