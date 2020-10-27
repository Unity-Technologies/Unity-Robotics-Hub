using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class ColorRGBA : Message
    {
        public const string RosMessageName = "std_msgs/ColorRGBA";

        public float r { get; set; }
        public float g { get; set; }
        public float b { get; set; }
        public float a { get; set; }

        public ColorRGBA()
        {
            this.r = 0.0f;
            this.g = 0.0f;
            this.b = 0.0f;
            this.a = 0.0f;
        }

        public ColorRGBA(float r, float g, float b, float a)
        {
            this.r = r;
            this.g = g;
            this.b = b;
            this.a = a;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.r));
            listOfSerializations.Add(BitConverter.GetBytes(this.g));
            listOfSerializations.Add(BitConverter.GetBytes(this.b));
            listOfSerializations.Add(BitConverter.GetBytes(this.a));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.r = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.g = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.b = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.a = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

    }
}
