using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class MultiEchoLaserScan : Message
    {
        public const string RosMessageName = "sensor_msgs/MultiEchoLaserScan";

        //  Single scan from a multi-echo planar laser range-finder
        // 
        //  If you have another ranging device with different behavior (e.g. a sonar
        //  array), please find or create a different message, since applications
        //  will make fairly laser-specific assumptions about this data
        public Header header { get; set; }
        //  timestamp in the header is the acquisition time of 
        //  the first ray in the scan.
        // 
        //  in frame frame_id, angles are measured around 
        //  the positive Z axis (counterclockwise, if Z is up)
        //  with zero angle being forward along the x axis
        public float angle_min { get; set; }
        //  start angle of the scan [rad]
        public float angle_max { get; set; }
        //  end angle of the scan [rad]
        public float angle_increment { get; set; }
        //  angular distance between measurements [rad]
        public float time_increment { get; set; }
        //  time between measurements [seconds] - if your scanner
        //  is moving, this will be used in interpolating position
        //  of 3d points
        public float scan_time { get; set; }
        //  time between scans [seconds]
        public float range_min { get; set; }
        //  minimum range value [m]
        public float range_max { get; set; }
        //  maximum range value [m]
        public LaserEcho[] ranges { get; set; }
        //  range data [m] (Note: NaNs, values < range_min or > range_max should be discarded)
        //  +Inf measurements are out of range
        //  -Inf measurements are too close to determine exact distance.
        public LaserEcho[] intensities { get; set; }
        //  intensity data [device-specific units].  If your
        //  device does not provide intensities, please leave
        //  the array empty.

        public MultiEchoLaserScan()
        {
            this.header = new Header();
            this.angle_min = 0.0f;
            this.angle_max = 0.0f;
            this.angle_increment = 0.0f;
            this.time_increment = 0.0f;
            this.scan_time = 0.0f;
            this.range_min = 0.0f;
            this.range_max = 0.0f;
            this.ranges = new LaserEcho[0];
            this.intensities = new LaserEcho[0];
        }

        public MultiEchoLaserScan(Header header, float angle_min, float angle_max, float angle_increment, float time_increment, float scan_time, float range_min, float range_max, LaserEcho[] ranges, LaserEcho[] intensities)
        {
            this.header = header;
            this.angle_min = angle_min;
            this.angle_max = angle_max;
            this.angle_increment = angle_increment;
            this.time_increment = time_increment;
            this.scan_time = scan_time;
            this.range_min = range_min;
            this.range_max = range_max;
            this.ranges = ranges;
            this.intensities = intensities;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.angle_min));
            listOfSerializations.Add(BitConverter.GetBytes(this.angle_max));
            listOfSerializations.Add(BitConverter.GetBytes(this.angle_increment));
            listOfSerializations.Add(BitConverter.GetBytes(this.time_increment));
            listOfSerializations.Add(BitConverter.GetBytes(this.scan_time));
            listOfSerializations.Add(BitConverter.GetBytes(this.range_min));
            listOfSerializations.Add(BitConverter.GetBytes(this.range_max));
            
            listOfSerializations.Add(BitConverter.GetBytes(ranges.Length));
            foreach(var entry in ranges)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(intensities.Length));
            foreach(var entry in intensities)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.angle_min = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.angle_max = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.angle_increment = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.time_increment = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.scan_time = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.range_min = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.range_max = BitConverter.ToSingle(data, offset);
            offset += 4;
            
            var rangesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.ranges= new LaserEcho[rangesArrayLength];
            for(var i =0; i <rangesArrayLength; i++)
            {
                this.ranges[i] = new LaserEcho();
                offset = this.ranges[i].Deserialize(data, offset);
            }
            
            var intensitiesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.intensities= new LaserEcho[intensitiesArrayLength];
            for(var i =0; i <intensitiesArrayLength; i++)
            {
                this.intensities[i] = new LaserEcho();
                offset = this.intensities[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
