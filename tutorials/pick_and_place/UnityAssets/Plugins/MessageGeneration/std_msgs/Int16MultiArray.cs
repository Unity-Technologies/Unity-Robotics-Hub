using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Int16MultiArray : Message
    {
        public const string RosMessageName = "std_msgs/Int16MultiArray";

        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayout layout { get; set; }
        //  specification of data layout
        public short[] data { get; set; }
        //  array of data

        public Int16MultiArray()
        {
            this.layout = new MultiArrayLayout();
            this.data = new short[0];
        }

        public Int16MultiArray(MultiArrayLayout layout, short[] data)
        {
            this.layout = layout;
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(layout.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            foreach(var entry in data)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.layout.Deserialize(data, offset);
            
            var dataArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.data= new short[dataArrayLength];
            for(var i =0; i <dataArrayLength; i++)
            {
                this.data[i] = BitConverter.ToInt16(data, offset);
                offset += 2;
            }

            return offset;
        }

    }
}
