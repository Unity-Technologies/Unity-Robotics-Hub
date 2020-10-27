using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class UInt32 : Message
    {
        public const string RosMessageName = "std_msgs/UInt32";

        public uint data { get; set; }

        public UInt32()
        {
            this.data = 0;
        }

        public UInt32(uint data)
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
            this.data = BitConverter.ToUInt32(data, offset);
            offset += 4;

            return offset;
        }

    }
}
