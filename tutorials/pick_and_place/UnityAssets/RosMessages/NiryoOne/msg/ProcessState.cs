using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class ProcessState : Message
    {
        public const string RosMessageName = "niryo_one_msgs/ProcessState";

        public string[] name { get; set; }
        public bool[] is_active { get; set; }

        public ProcessState()
        {
            this.name = new string[0];
            this.is_active = new bool[0];
        }

        public ProcessState(string[] name, bool[] is_active)
        {
            this.name = name;
            this.is_active = is_active;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(name.Length));
            foreach(var entry in name)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(is_active.Length));
            foreach(var entry in is_active)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var nameArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.name= new string[nameArrayLength];
            for(var i =0; i <nameArrayLength; i++)
            {
                var nameStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.name[i] = DeserializeString(data, offset, nameStringBytesLength);
                offset += nameStringBytesLength;
            }
            
            var is_activeArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.is_active= new bool[is_activeArrayLength];
            for(var i =0; i <is_activeArrayLength; i++)
            {
                this.is_active[i] = BitConverter.ToBoolean(data, offset);
                offset += 1;
            }

            return offset;
        }

    }
}
