using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class ByteMultiArray : Message
    {
        public const string RosMessageName = "std_msgs/ByteMultiArray";

        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayout layout { get; set; }
        //  specification of data layout
        public sbyte[] data { get; set; }
        //  array of data

        public ByteMultiArray()
        {
            this.layout = new MultiArrayLayout();
            this.data = new sbyte[0];
        }

        public ByteMultiArray(MultiArrayLayout layout, sbyte[] data)
        {
            this.layout = layout;
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(layout.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            listOfSerializations.Add((byte[]) (Array)this.data);

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.layout.Deserialize(data, offset);
            
            var dataArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.data= new sbyte[dataArrayLength];
            for(var i =0; i <dataArrayLength; i++)
            {
                this.data[i] = (sbyte)data[offset];
                offset += 1;
            }

            return offset;
        }

    }
}
