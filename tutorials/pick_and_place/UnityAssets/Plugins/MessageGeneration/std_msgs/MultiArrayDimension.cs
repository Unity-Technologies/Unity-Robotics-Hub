using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class MultiArrayDimension : Message
    {
        public const string RosMessageName = "std_msgs/MultiArrayDimension";

        public string label { get; set; }
        //  label of given dimension
        public uint size { get; set; }
        //  size of given dimension (in type units)
        public uint stride { get; set; }
        //  stride of given dimension

        public MultiArrayDimension()
        {
            this.label = "";
            this.size = 0;
            this.stride = 0;
        }

        public MultiArrayDimension(string label, uint size, uint stride)
        {
            this.label = label;
            this.size = size;
            this.stride = stride;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.label));
            listOfSerializations.Add(BitConverter.GetBytes(this.size));
            listOfSerializations.Add(BitConverter.GetBytes(this.stride));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var stringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.label = DeserializeString(data, offset, stringBytesLength);
            offset += stringBytesLength;
            this.size = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.stride = BitConverter.ToUInt32(data, offset);
            offset += 4;

            return offset;
        }

    }
}
