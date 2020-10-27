using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Sensor
{
    public class PointField : Message
    {
        public const string RosMessageName = "sensor_msgs/PointField";

        //  This message holds the description of one point entry in the
        //  PointCloud2 message format.
        public const byte INT8 = 1;
        public const byte UINT8 = 2;
        public const byte INT16 = 3;
        public const byte UINT16 = 4;
        public const byte INT32 = 5;
        public const byte UINT32 = 6;
        public const byte FLOAT32 = 7;
        public const byte FLOAT64 = 8;
        public string name { get; set; }
        //  Name of field
        public uint offset { get; set; }
        //  Offset from start of point struct
        public byte datatype { get; set; }
        //  Datatype enumeration, see above
        public uint count { get; set; }
        //  How many elements in the field

        public PointField()
        {
            this.name = "";
            this.offset = 0;
            this.datatype = 0;
            this.count = 0;
        }

        public PointField(string name, uint offset, byte datatype, uint count)
        {
            this.name = name;
            this.offset = offset;
            this.datatype = datatype;
            this.count = count;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.Add(BitConverter.GetBytes(this.offset));
            listOfSerializations.Add(BitConverter.GetBytes(this.datatype));
            listOfSerializations.Add(BitConverter.GetBytes(this.count));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            this.offset = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.datatype = data[offset];;
            offset += 1;
            this.count = BitConverter.ToUInt32(data, offset);
            offset += 4;

            return offset;
        }

    }
}
