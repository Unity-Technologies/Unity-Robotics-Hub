using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class Range : Message
    {
        public const string RosMessageName = "sensor_msgs/Range";

        //  Single range reading from an active ranger that emits energy and reports
        //  one range reading that is valid along an arc at the distance measured. 
        //  This message is  not appropriate for laser scanners. See the LaserScan
        //  message if you are working with a laser scanner.
        //  This message also can represent a fixed-distance (binary) ranger.  This
        //  sensor will have min_range===max_range===distance of detection.
        //  These sensors follow REP 117 and will output -Inf if the object is detected
        //  and +Inf if the object is outside of the detection range.
        public Header header { get; set; }
        //  timestamp in the header is the time the ranger
        //  returned the distance reading
        //  Radiation type enums
        //  If you want a value added to this list, send an email to the ros-users list
        public const byte ULTRASOUND = 0;
        public const byte INFRARED = 1;
        public byte radiation_type { get; set; }
        //  the type of radiation used by the sensor
        //  (sound, IR, etc) [enum]
        public float field_of_view { get; set; }
        //  the size of the arc that the distance reading is
        //  valid for [rad]
        //  the object causing the range reading may have
        //  been anywhere within -field_of_view/2 and
        //  field_of_view/2 at the measured range. 
        //  0 angle corresponds to the x-axis of the sensor.
        public float min_range { get; set; }
        //  minimum range value [m]
        public float max_range { get; set; }
        //  maximum range value [m]
        //  Fixed distance rangers require min_range==max_range
        public float range { get; set; }
        //  range data [m]
        //  (Note: values < range_min or > range_max
        //  should be discarded)
        //  Fixed distance rangers only output -Inf or +Inf.
        //  -Inf represents a detection within fixed distance.
        //  (Detection too close to the sensor to quantify)
        //  +Inf represents no detection within the fixed distance.
        //  (Object out of range)

        public Range()
        {
            this.header = new Header();
            this.radiation_type = 0;
            this.field_of_view = 0.0f;
            this.min_range = 0.0f;
            this.max_range = 0.0f;
            this.range = 0.0f;
        }

        public Range(Header header, byte radiation_type, float field_of_view, float min_range, float max_range, float range)
        {
            this.header = header;
            this.radiation_type = radiation_type;
            this.field_of_view = field_of_view;
            this.min_range = min_range;
            this.max_range = max_range;
            this.range = range;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.radiation_type));
            listOfSerializations.Add(BitConverter.GetBytes(this.field_of_view));
            listOfSerializations.Add(BitConverter.GetBytes(this.min_range));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_range));
            listOfSerializations.Add(BitConverter.GetBytes(this.range));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.radiation_type = data[offset];;
            offset += 1;
            this.field_of_view = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.min_range = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.max_range = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.range = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

    }
}
