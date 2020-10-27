using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class MultiArrayLayout : Message
    {
        public const string RosMessageName = "std_msgs/MultiArrayLayout";

        //  The multiarray declares a generic multi-dimensional array of a
        //  particular data type.  Dimensions are ordered from outer most
        //  to inner most.
        public MultiArrayDimension[] dim { get; set; }
        //  Array of dimension properties
        public uint data_offset { get; set; }
        //  padding elements at front of data
        //  Accessors should ALWAYS be written in terms of dimension stride
        //  and specified outer-most dimension first.
        //  
        //  multiarray(i,j,k) = data[data_offset + dim_stride[1]*i + dim_stride[2]*j + k]
        // 
        //  A standard, 3-channel 640x480 image with interleaved color channels
        //  would be specified as:
        // 
        //  dim[0].label  = "height"
        //  dim[0].size   = 480
        //  dim[0].stride = 3*640*480 = 921600  (note dim[0] stride is just size of image)
        //  dim[1].label  = "width"
        //  dim[1].size   = 640
        //  dim[1].stride = 3*640 = 1920
        //  dim[2].label  = "channel"
        //  dim[2].size   = 3
        //  dim[2].stride = 3
        // 
        //  multiarray(i,j,k) refers to the ith row, jth column, and kth channel.

        public MultiArrayLayout()
        {
            this.dim = new MultiArrayDimension[0];
            this.data_offset = 0;
        }

        public MultiArrayLayout(MultiArrayDimension[] dim, uint data_offset)
        {
            this.dim = dim;
            this.data_offset = data_offset;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(dim.Length));
            foreach(var entry in dim)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.data_offset));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var dimArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.dim= new MultiArrayDimension[dimArrayLength];
            for(var i =0; i <dimArrayLength; i++)
            {
                this.dim[i] = new MultiArrayDimension();
                offset = this.dim[i].Deserialize(data, offset);
            }
            this.data_offset = BitConverter.ToUInt32(data, offset);
            offset += 4;

            return offset;
        }

    }
}
