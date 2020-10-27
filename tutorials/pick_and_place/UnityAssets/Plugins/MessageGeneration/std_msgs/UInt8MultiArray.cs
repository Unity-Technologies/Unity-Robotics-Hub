using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class UInt8MultiArray : Message
    {
        public const string RosMessageName = "std_msgs/UInt8MultiArray";

        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayout layout { get; set; }
        //  specification of data layout
        public byte[] data { get; set; }
        //  array of data

        public UInt8MultiArray()
        {
            this.layout = new MultiArrayLayout();
            this.data = new byte[0];
        }

        public UInt8MultiArray(MultiArrayLayout layout, byte[] data)
        {
            this.layout = layout;
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(layout.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            listOfSerializations.Add(this.data);

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.layout.Deserialize(data, offset);
            
            var dataArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.data= new byte[dataArrayLength];
            for(var i =0; i <dataArrayLength; i++)
            {
                this.data[i] = data[offset];
                offset += 1;
            }

            return offset;
        }

    }
}
