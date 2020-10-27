using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class PointCloud2 : Message
    {
        public const string RosMessageName = "sensor_msgs/PointCloud2";

        //  This message holds a collection of N-dimensional points, which may
        //  contain additional information such as normals, intensity, etc. The
        //  point data is stored as a binary blob, its layout described by the
        //  contents of the "fields" array.
        //  The point cloud data may be organized 2d (image-like) or 1d
        //  (unordered). Point clouds organized as 2d images may be produced by
        //  camera depth sensors such as stereo or time-of-flight.
        //  Time of sensor data acquisition, and the coordinate frame ID (for 3d
        //  points).
        public Header header { get; set; }
        //  2D structure of the point cloud. If the cloud is unordered, height is
        //  1 and width is the length of the point cloud.
        public uint height { get; set; }
        public uint width { get; set; }
        //  Describes the channels and their layout in the binary data blob.
        public PointField[] fields { get; set; }
        public bool is_bigendian { get; set; }
        //  Is this data bigendian?
        public uint point_step { get; set; }
        //  Length of a point in bytes
        public uint row_step { get; set; }
        //  Length of a row in bytes
        public byte[] data { get; set; }
        //  Actual point data, size is (row_step*height)
        public bool is_dense { get; set; }
        //  True if there are no invalid points

        public PointCloud2()
        {
            this.header = new Header();
            this.height = 0;
            this.width = 0;
            this.fields = new PointField[0];
            this.is_bigendian = false;
            this.point_step = 0;
            this.row_step = 0;
            this.data = new byte[0];
            this.is_dense = false;
        }

        public PointCloud2(Header header, uint height, uint width, PointField[] fields, bool is_bigendian, uint point_step, uint row_step, byte[] data, bool is_dense)
        {
            this.header = header;
            this.height = height;
            this.width = width;
            this.fields = fields;
            this.is_bigendian = is_bigendian;
            this.point_step = point_step;
            this.row_step = row_step;
            this.data = data;
            this.is_dense = is_dense;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.height));
            listOfSerializations.Add(BitConverter.GetBytes(this.width));
            
            listOfSerializations.Add(BitConverter.GetBytes(fields.Length));
            foreach(var entry in fields)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.is_bigendian));
            listOfSerializations.Add(BitConverter.GetBytes(this.point_step));
            listOfSerializations.Add(BitConverter.GetBytes(this.row_step));
            
            listOfSerializations.Add(BitConverter.GetBytes(data.Length));
            listOfSerializations.Add(this.data);
            listOfSerializations.Add(BitConverter.GetBytes(this.is_dense));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.height = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.width = BitConverter.ToUInt32(data, offset);
            offset += 4;
            
            var fieldsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.fields= new PointField[fieldsArrayLength];
            for(var i =0; i <fieldsArrayLength; i++)
            {
                this.fields[i] = new PointField();
                offset = this.fields[i].Deserialize(data, offset);
            }
            this.is_bigendian = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.point_step = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.row_step = BitConverter.ToUInt32(data, offset);
            offset += 4;
            
            var dataArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.data= new byte[dataArrayLength];
            for(var i =0; i <dataArrayLength; i++)
            {
                this.data[i] = data[offset];
                offset += 1;
            }
            this.is_dense = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
