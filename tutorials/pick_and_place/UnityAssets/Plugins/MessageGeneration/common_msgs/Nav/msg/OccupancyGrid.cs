using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Nav
{
    public class OccupancyGrid : Message
    {
        public const string RosMessageName = "nav_msgs/OccupancyGrid";

        //  This represents a 2-D grid map, in which each cell represents the probability of
        //  occupancy.
        public Header header { get; set; }
        // MetaData for the map
        public MapMetaData info { get; set; }
        //  The map data, in row-major order, starting with (0,0).  Occupancy
        //  probabilities are in the range [0,100].  Unknown is -1.
        public sbyte[] data { get; set; }

        public OccupancyGrid()
        {
            this.header = new Header();
            this.info = new MapMetaData();
            this.data = new sbyte[0];
        }

        public OccupancyGrid(Header header, MapMetaData info, sbyte[] data)
        {
            this.header = header;
            this.info = info;
            this.data = data;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(info.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            listOfSerializations.Add((byte[]) (Array)this.data);

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.info.Deserialize(data, offset);
            
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
