using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Sensor
{
    public class JoyFeedback : Message
    {
        public const string RosMessageName = "sensor_msgs/JoyFeedback";

        //  Declare of the type of feedback
        public const byte TYPE_LED = 0;
        public const byte TYPE_RUMBLE = 1;
        public const byte TYPE_BUZZER = 2;
        public byte type { get; set; }
        //  This will hold an id number for each type of each feedback.
        //  Example, the first led would be id=0, the second would be id=1
        public byte id { get; set; }
        //  Intensity of the feedback, from 0.0 to 1.0, inclusive.  If device is
        //  actually binary, driver should treat 0<=x<0.5 as off, 0.5<=x<=1 as on.
        public float intensity { get; set; }

        public JoyFeedback()
        {
            this.type = 0;
            this.id = 0;
            this.intensity = 0.0f;
        }

        public JoyFeedback(byte type, byte id, float intensity)
        {
            this.type = type;
            this.id = id;
            this.intensity = intensity;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.type));
            listOfSerializations.Add(BitConverter.GetBytes(this.id));
            listOfSerializations.Add(BitConverter.GetBytes(this.intensity));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.type = data[offset];;
            offset += 1;
            this.id = data[offset];;
            offset += 1;
            this.intensity = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

    }
}
