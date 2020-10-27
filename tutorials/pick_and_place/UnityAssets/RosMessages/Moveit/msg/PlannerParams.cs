using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class PlannerParams : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PlannerParams";

        //  parameter names (same size as values)
        public string[] keys { get; set; }
        //  parameter values (same size as keys)
        public string[] values { get; set; }
        //  parameter description (can be empty)
        public string[] descriptions { get; set; }

        public PlannerParams()
        {
            this.keys = new string[0];
            this.values = new string[0];
            this.descriptions = new string[0];
        }

        public PlannerParams(string[] keys, string[] values, string[] descriptions)
        {
            this.keys = keys;
            this.values = values;
            this.descriptions = descriptions;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(keys.Length));
            foreach(var entry in keys)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(values.Length));
            foreach(var entry in values)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(descriptions.Length));
            foreach(var entry in descriptions)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var keysArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.keys= new string[keysArrayLength];
            for(var i =0; i <keysArrayLength; i++)
            {
                var keysStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.keys[i] = DeserializeString(data, offset, keysStringBytesLength);
                offset += keysStringBytesLength;
            }
            
            var valuesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.values= new string[valuesArrayLength];
            for(var i =0; i <valuesArrayLength; i++)
            {
                var valuesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.values[i] = DeserializeString(data, offset, valuesStringBytesLength);
                offset += valuesStringBytesLength;
            }
            
            var descriptionsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.descriptions= new string[descriptionsArrayLength];
            for(var i =0; i <descriptionsArrayLength; i++)
            {
                var descriptionsStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.descriptions[i] = DeserializeString(data, offset, descriptionsStringBytesLength);
                offset += descriptionsStringBytesLength;
            }

            return offset;
        }

    }
}
