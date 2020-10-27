using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Diagnostic
{
    public class KeyValue : Message
    {
        public const string RosMessageName = "diagnostic_msgs/KeyValue";

        public string key { get; set; }
        //  what to label this value when viewing
        public string value { get; set; }
        //  a value to track over time

        public KeyValue()
        {
            this.key = "";
            this.value = "";
        }

        public KeyValue(string key, string value)
        {
            this.key = key;
            this.value = value;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.key));
            listOfSerializations.Add(SerializeString(this.value));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var keyStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.key = DeserializeString(data, offset, keyStringBytesLength);
            offset += keyStringBytesLength;
            var valueStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.value = DeserializeString(data, offset, valueStringBytesLength);
            offset += valueStringBytesLength;

            return offset;
        }

    }
}
