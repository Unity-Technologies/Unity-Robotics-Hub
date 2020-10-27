using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class AllowedCollisionMatrix : Message
    {
        public const string RosMessageName = "moveit_msgs-master/AllowedCollisionMatrix";

        //  The list of entry names in the matrix
        public string[] entry_names { get; set; }
        //  The individual entries in the allowed collision matrix
        //  square, symmetric, with same order as entry_names
        public AllowedCollisionEntry[] entry_values { get; set; }
        //  In addition to the collision matrix itself, we also have 
        //  the default entry value for each entry name.
        //  If the allowed collision flag is queried for a pair of names (n1, n2)
        //  that is not found in the collision matrix itself, the value of
        //  the collision flag is considered to be that of the entry (n1 or n2)
        //  specified in the list below. If both n1 and n2 are found in the list 
        //  of defaults, the result is computed with an AND operation
        public string[] default_entry_names { get; set; }
        public bool[] default_entry_values { get; set; }

        public AllowedCollisionMatrix()
        {
            this.entry_names = new string[0];
            this.entry_values = new AllowedCollisionEntry[0];
            this.default_entry_names = new string[0];
            this.default_entry_values = new bool[0];
        }

        public AllowedCollisionMatrix(string[] entry_names, AllowedCollisionEntry[] entry_values, string[] default_entry_names, bool[] default_entry_values)
        {
            this.entry_names = entry_names;
            this.entry_values = entry_values;
            this.default_entry_names = default_entry_names;
            this.default_entry_values = default_entry_values;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(entry_names.Length));
            foreach(var entry in entry_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(entry_values.Length));
            foreach(var entry in entry_values)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(default_entry_names.Length));
            foreach(var entry in default_entry_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(default_entry_values.Length));
            foreach(var entry in default_entry_values)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var entry_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.entry_names= new string[entry_namesArrayLength];
            for(var i =0; i <entry_namesArrayLength; i++)
            {
                var entry_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.entry_names[i] = DeserializeString(data, offset, entry_namesStringBytesLength);
                offset += entry_namesStringBytesLength;
            }
            
            var entry_valuesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.entry_values= new AllowedCollisionEntry[entry_valuesArrayLength];
            for(var i =0; i <entry_valuesArrayLength; i++)
            {
                this.entry_values[i] = new AllowedCollisionEntry();
                offset = this.entry_values[i].Deserialize(data, offset);
            }
            
            var default_entry_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.default_entry_names= new string[default_entry_namesArrayLength];
            for(var i =0; i <default_entry_namesArrayLength; i++)
            {
                var default_entry_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.default_entry_names[i] = DeserializeString(data, offset, default_entry_namesStringBytesLength);
                offset += default_entry_namesStringBytesLength;
            }
            
            var default_entry_valuesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.default_entry_values= new bool[default_entry_valuesArrayLength];
            for(var i =0; i <default_entry_valuesArrayLength; i++)
            {
                this.default_entry_values[i] = BitConverter.ToBoolean(data, offset);
                offset += 1;
            }

            return offset;
        }

    }
}
