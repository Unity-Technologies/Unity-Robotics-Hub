using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class FluidPressure : Message
    {
        public const string RosMessageName = "sensor_msgs/FluidPressure";

        //  Single pressure reading.  This message is appropriate for measuring the
        //  pressure inside of a fluid (air, water, etc).  This also includes
        //  atmospheric or barometric pressure.
        //  This message is not appropriate for force/pressure contact sensors.
        public Header header { get; set; }
        //  timestamp of the measurement
        //  frame_id is the location of the pressure sensor
        public double fluid_pressure { get; set; }
        //  Absolute pressure reading in Pascals.
        public double variance { get; set; }
        //  0 is interpreted as variance unknown

        public FluidPressure()
        {
            this.header = new Header();
            this.fluid_pressure = 0.0;
            this.variance = 0.0;
        }

        public FluidPressure(Header header, double fluid_pressure, double variance)
        {
            this.header = header;
            this.fluid_pressure = fluid_pressure;
            this.variance = variance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.fluid_pressure));
            listOfSerializations.Add(BitConverter.GetBytes(this.variance));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            this.fluid_pressure = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.variance = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
