using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Sensor
{
    public class PointCloud : Message
    {
        public const string RosMessageName = "sensor_msgs/PointCloud";

        //  This message holds a collection of 3d points, plus optional additional
        //  information about each point.
        //  Time of sensor data acquisition, coordinate frame ID.
        public Header header { get; set; }
        //  Array of 3d points. Each Point32 should be interpreted as a 3d point
        //  in the frame given in the header.
        public Point32[] points { get; set; }
        //  Each channel should have the same number of elements as points array,
        //  and the data in each channel should correspond 1:1 with each point.
        //  Channel names in common practice are listed in ChannelFloat32.msg.
        public ChannelFloat32[] channels { get; set; }

        public PointCloud()
        {
            this.header = new Header();
            this.points = new Point32[0];
            this.channels = new ChannelFloat32[0];
        }

        public PointCloud(Header header, Point32[] points, ChannelFloat32[] channels)
        {
            this.header = header;
            this.points = points;
            this.channels = channels;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(points.Length));
            foreach(var entry in points)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(channels.Length));
            foreach(var entry in channels)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var pointsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.points= new Point32[pointsArrayLength];
            for(var i =0; i <pointsArrayLength; i++)
            {
                this.points[i] = new Point32();
                offset = this.points[i].Deserialize(data, offset);
            }
            
            var channelsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.channels= new ChannelFloat32[channelsArrayLength];
            for(var i =0; i <channelsArrayLength; i++)
            {
                this.channels[i] = new ChannelFloat32();
                offset = this.channels[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
