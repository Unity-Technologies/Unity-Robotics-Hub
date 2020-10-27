using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class Temperature : Message
    {
        public const string RosMessageName = "sensor_msgs/Temperature";

        //  Single temperature reading.
        public Header header { get; set; }
        //  timestamp is the time the temperature was measured
        //  frame_id is the location of the temperature reading
        public double temperature { get; set; }
        //  Measurement of the Temperature in Degrees Celsius
        public double variance { get; set; }
        //  0 is interpreted as variance unknown

        public Temperature()
        {
            this.header = new Header();
            this.temperature = 0.0;
            this.variance = 0.0;
        }

        public Temperature(Header header, double temperature, double variance)
        {
            this.header = header;
            this.temperature = temperature;
            this.variance = variance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.temperature));
            listOfSerializations.Add(BitConverter.GetBytes(this.variance));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.temperature = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.variance = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
