using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Int64 : Message
    {
        public const string RosMessageName = "std_msgs/Int64";

        public long data { get; set; }

        public Int64()
        {
            this.data = 0;
        }

        public Int64(long data)
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
            this.data = BitConverter.ToInt64(data, offset);
            offset += 8;

            return offset;
        }

    }
}
