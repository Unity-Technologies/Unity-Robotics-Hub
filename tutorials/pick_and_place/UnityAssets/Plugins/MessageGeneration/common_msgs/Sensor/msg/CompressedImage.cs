using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class CompressedImage : Message
    {
        public const string RosMessageName = "sensor_msgs/CompressedImage";

        //  This message contains a compressed image
        public Header header { get; set; }
        //  Header timestamp should be acquisition time of image
        //  Header frame_id should be optical frame of camera
        //  origin of frame should be optical center of camera
        //  +x should point to the right in the image
        //  +y should point down in the image
        //  +z should point into to plane of the image
        public string format { get; set; }
        //  Specifies the format of the data
        //    Acceptable values:
        //      jpeg, png
        public byte[] data { get; set; }
        //  Compressed image buffer

        public CompressedImage()
        {
            this.header = new Header();
            this.format = "";
            this.data = new byte[0];
        }

        public CompressedImage(Header header, string format, byte[] data)
        {
            this.header = header;
            this.format = format;
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.format));
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            listOfSerializations.Add(this.data);

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            var formatStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.format = DeserializeString(data, offset, formatStringBytesLength);
            offset += formatStringBytesLength;
            
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
