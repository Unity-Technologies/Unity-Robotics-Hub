using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Nav
{
    public class MapMetaData : Message
    {
        public const string RosMessageName = "nav_msgs/MapMetaData";

        //  This hold basic information about the characterists of the OccupancyGrid
        //  The time at which the map was loaded
        public Time map_load_time { get; set; }
        //  The map resolution [m/cell]
        public float resolution { get; set; }
        //  Map width [cells]
        public uint width { get; set; }
        //  Map height [cells]
        public uint height { get; set; }
        //  The origin of the map [m, m, rad].  This is the real-world pose of the
        //  cell (0,0) in the map.
        public Pose origin { get; set; }

        public MapMetaData()
        {
            this.map_load_time = new Time();
            this.resolution = 0.0f;
            this.width = 0;
            this.height = 0;
            this.origin = new Pose();
        }

        public MapMetaData(Time map_load_time, float resolution, uint width, uint height, Pose origin)
        {
            this.map_load_time = map_load_time;
            this.resolution = resolution;
            this.width = width;
            this.height = height;
            this.origin = origin;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(map_load_time.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.resolution));
            listOfSerializations.Add(BitConverter.GetBytes(this.width));
            listOfSerializations.Add(BitConverter.GetBytes(this.height));
            listOfSerializations.AddRange(origin.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.map_load_time.Deserialize(data, offset);
            this.resolution = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.width = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.height = BitConverter.ToUInt32(data, offset);
            offset += 4;
            offset = this.origin.Deserialize(data, offset);

            return offset;
        }

    }
}
