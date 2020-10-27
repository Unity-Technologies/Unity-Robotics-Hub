using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Octomap
{
    public class Octomap : Message
    {
        public const string RosMessageName = "octomap_msgs-melodic-devel/Octomap";

        //  A 3D map in binary format, as Octree
        public Header header { get; set; }
        //  Flag to denote a binary (only free/occupied) or full occupancy octree (.bt/.ot file)
        public bool binary { get; set; }
        //  Class id of the contained octree 
        public string id { get; set; }
        //  Resolution (in m) of the smallest octree nodes
        public double resolution { get; set; }
        //  binary serialization of octree, use conversions.h to read and write octrees
        public sbyte[] data { get; set; }

        public Octomap()
        {
            this.header = new Header();
            this.binary = false;
            this.id = "";
            this.resolution = 0.0;
            this.data = new sbyte[0];
        }

        public Octomap(Header header, bool binary, string id, double resolution, sbyte[] data)
        {
            this.header = header;
            this.binary = binary;
            this.id = id;
            this.resolution = resolution;
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.binary));
            listOfSerializations.Add(SerializeString(this.id));
            listOfSerializations.Add(BitConverter.GetBytes(this.resolution));
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            listOfSerializations.Add((byte[]) (Array)this.data);

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.binary = BitConverter.ToBoolean(data, offset);
            offset += 1;
            var idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.id = DeserializeString(data, offset, idStringBytesLength);
            offset += idStringBytesLength;
            this.resolution = BitConverter.ToDouble(data, offset);
            offset += 8;
            
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
