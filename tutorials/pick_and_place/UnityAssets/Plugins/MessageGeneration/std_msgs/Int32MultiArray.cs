using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Int32MultiArray : Message
    {
        public const string RosMessageName = "std_msgs/Int32MultiArray";

        //  Please look at the MultiArrayLayout message definition for
        //  documentation on all multiarrays.
        public MultiArrayLayout layout { get; set; }
        //  specification of data layout
        public int[] data { get; set; }
        //  array of data

        public Int32MultiArray()
        {
            this.layout = new MultiArrayLayout();
            this.data = new int[0];
        }

        public Int32MultiArray(MultiArrayLayout layout, int[] data)
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
            this.data= new int[dataArrayLength];
            for(var i =0; i <dataArrayLength; i++)
            {
                this.data[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
