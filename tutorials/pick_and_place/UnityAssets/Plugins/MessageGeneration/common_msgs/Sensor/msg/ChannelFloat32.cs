using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Sensor
{
    public class ChannelFloat32 : Message
    {
        public const string RosMessageName = "sensor_msgs/ChannelFloat32";

        //  This message is used by the PointCloud message to hold optional data
        //  associated with each point in the cloud. The length of the values
        //  array should be the same as the length of the points array in the
        //  PointCloud, and each value should be associated with the corresponding
        //  point.
        //  Channel names in existing practice include:
        //    "u", "v" - row and column (respectively) in the left stereo image.
        //               This is opposite to usual conventions but remains for
        //               historical reasons. The newer PointCloud2 message has no
        //               such problem.
        //    "rgb" - For point clouds produced by color stereo cameras. uint8
        //            (R,G,B) values packed into the least significant 24 bits,
        //            in order.
        //    "intensity" - laser or pixel intensity.
        //    "distance"
        //  The channel name should give semantics of the channel (e.g.
        //  "intensity" instead of "value").
        public string name { get; set; }
        //  The values array should be 1-1 with the elements of the associated
        //  PointCloud.
        public float[] values { get; set; }

        public ChannelFloat32()
        {
            this.name = "";
            this.values = new float[0];
        }

        public ChannelFloat32(string name, float[] values)
        {
            this.name = name;
            this.values = values;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            
            listOfSerializations.Add(BitConverter.GetBytes(values.Length));
            foreach(var entry in values)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            
            var valuesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.values= new float[valuesArrayLength];
            for(var i =0; i <valuesArrayLength; i++)
            {
                this.values[i] = BitConverter.ToSingle(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
