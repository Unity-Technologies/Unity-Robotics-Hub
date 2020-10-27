using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Bool : Message
    {
        public const string RosMessageName = "std_msgs/Bool";

        public bool data { get; set; }

        public Bool()
        {
            this.data = false;
        }

        public Bool(bool data)
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
            this.data = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
