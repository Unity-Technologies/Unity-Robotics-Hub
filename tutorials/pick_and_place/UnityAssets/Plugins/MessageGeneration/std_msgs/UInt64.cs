using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class UInt64 : Message
    {
        public const string RosMessageName = "std_msgs/UInt64";

        public ulong data { get; set; }

        public UInt64()
        {
            this.data = 0;
        }

        public UInt64(ulong data)
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
            this.data = BitConverter.ToUInt64(data, offset);
            offset += 8;

            return offset;
        }

    }
}
