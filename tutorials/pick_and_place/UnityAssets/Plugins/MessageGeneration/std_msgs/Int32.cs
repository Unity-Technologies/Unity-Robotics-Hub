using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Int32 : Message
    {
        public const string RosMessageName = "std_msgs/Int32";

        public int data { get; set; }

        public Int32()
        {
            this.data = 0;
        }

        public Int32(int data)
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
            this.data = BitConverter.ToInt32(data, offset);
            offset += 4;

            return offset;
        }

    }
}
