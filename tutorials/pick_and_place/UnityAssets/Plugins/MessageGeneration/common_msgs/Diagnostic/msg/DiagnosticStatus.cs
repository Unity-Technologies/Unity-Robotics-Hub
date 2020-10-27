using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Diagnostic
{
    public class DiagnosticStatus : Message
    {
        public const string RosMessageName = "diagnostic_msgs/DiagnosticStatus";

        //  This message holds the status of an individual component of the robot.
        //  
        //  Possible levels of operations
        public const sbyte OK = 0;
        public const sbyte WARN = 1;
        public const sbyte ERROR = 2;
        public const sbyte STALE = 3;
        public sbyte level { get; set; }
        //  level of operation enumerated above 
        public string name { get; set; }
        //  a description of the test/component reporting
        public string message { get; set; }
        //  a description of the status
        public string hardware_id { get; set; }
        //  a hardware unique string
        public KeyValue[] values { get; set; }
        //  an array of values associated with the status

        public DiagnosticStatus()
        {
            this.level = 0;
            this.name = "";
            this.message = "";
            this.hardware_id = "";
            this.values = new KeyValue[0];
        }

        public DiagnosticStatus(sbyte level, string name, string message, string hardware_id, KeyValue[] values)
        {
            this.level = level;
            this.name = name;
            this.message = message;
            this.hardware_id = hardware_id;
            this.values = values;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.level));
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.Add(SerializeString(this.message));
            listOfSerializations.Add(SerializeString(this.hardware_id));
            
            listOfSerializations.Add(BitConverter.GetBytes(values.Length));
            foreach(var entry in values)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.level = (sbyte)data[offset];;
            offset += 1;
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            var messageStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.message = DeserializeString(data, offset, messageStringBytesLength);
            offset += messageStringBytesLength;
            var hardware_idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.hardware_id = DeserializeString(data, offset, hardware_idStringBytesLength);
            offset += hardware_idStringBytesLength;
            
            var valuesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.values= new KeyValue[valuesArrayLength];
            for(var i =0; i <valuesArrayLength; i++)
            {
                this.values[i] = new KeyValue();
                offset = this.values[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
