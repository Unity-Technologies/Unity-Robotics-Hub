using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class AllowedCollisionEntry : Message
    {
        public const string RosMessageName = "moveit_msgs-master/AllowedCollisionEntry";

        //  whether or not collision checking is enabled
        public bool[] enabled { get; set; }

        public AllowedCollisionEntry()
        {
            this.enabled = new bool[0];
        }

        public AllowedCollisionEntry(bool[] enabled)
        {
            this.enabled = enabled;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(enabled.Length));
            foreach(var entry in enabled)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var enabledArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.enabled= new bool[enabledArrayLength];
            for(var i =0; i <enabledArrayLength; i++)
            {
                this.enabled[i] = BitConverter.ToBoolean(data, offset);
                offset += 1;
            }

            return offset;
        }

    }
}
