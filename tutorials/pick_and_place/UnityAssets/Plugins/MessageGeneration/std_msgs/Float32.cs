using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Float32 : Message
    {
        public const string RosMessageName = "std_msgs/Float32";

        public float data { get; set; }

        public Float32()
        {
            this.data = 0.0f;
        }

        public Float32(float data)
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
            this.data = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

    }
}
