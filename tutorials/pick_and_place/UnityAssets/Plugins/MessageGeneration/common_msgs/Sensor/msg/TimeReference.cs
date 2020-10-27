using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class TimeReference : Message
    {
        public const string RosMessageName = "sensor_msgs/TimeReference";

        //  Measurement from an external time source not actively synchronized with the system clock.
        public Header header { get; set; }
        //  stamp is system time for which measurement was valid
        //  frame_id is not used 
        public Time time_ref { get; set; }
        //  corresponding time from this external source
        public string source { get; set; }
        //  (optional) name of time source

        public TimeReference()
        {
            this.header = new Header();
            this.time_ref = new Time();
            this.source = "";
        }

        public TimeReference(Header header, Time time_ref, string source)
        {
            this.header = header;
            this.time_ref = time_ref;
            this.source = source;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(time_ref.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.source));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.time_ref.Deserialize(data, offset);
            var sourceStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.source = DeserializeString(data, offset, sourceStringBytesLength);
            offset += sourceStringBytesLength;

            return offset;
        }

    }
}
