using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class RelativeHumidity : Message
    {
        public const string RosMessageName = "sensor_msgs/RelativeHumidity";

        //  Single reading from a relative humidity sensor.  Defines the ratio of partial
        //  pressure of water vapor to the saturated vapor pressure at a temperature.
        public Header header { get; set; }
        //  timestamp of the measurement
        //  frame_id is the location of the humidity sensor
        public double relative_humidity { get; set; }
        //  Expression of the relative humidity
        //  from 0.0 to 1.0.
        //  0.0 is no partial pressure of water vapor
        //  1.0 represents partial pressure of saturation
        public double variance { get; set; }
        //  0 is interpreted as variance unknown

        public RelativeHumidity()
        {
            this.header = new Header();
            this.relative_humidity = 0.0;
            this.variance = 0.0;
        }

        public RelativeHumidity(Header header, double relative_humidity, double variance)
        {
            this.header = header;
            this.relative_humidity = relative_humidity;
            this.variance = variance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.relative_humidity));
            listOfSerializations.Add(BitConverter.GetBytes(this.variance));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.relative_humidity = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.variance = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
