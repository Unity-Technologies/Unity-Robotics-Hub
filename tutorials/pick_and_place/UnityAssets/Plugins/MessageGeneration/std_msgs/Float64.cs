using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Float64 : Message
    {
        public const string RosMessageName = "std_msgs/Float64";

        public double data { get; set; }

        public Float64()
        {
            this.data = 0.0;
        }

        public Float64(double data)
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
            this.data = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
