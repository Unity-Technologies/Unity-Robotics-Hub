using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Int16 : Message
    {
        public const string RosMessageName = "std_msgs/Int16";

        public short data { get; set; }

        public Int16()
        {
            this.data = 0;
        }

        public Int16(short data)
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
            this.data = BitConverter.ToInt16(data, offset);
            offset += 2;

            return offset;
        }

    }
}
