using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class SequenceAutorunStatus : Message
    {
        public const string RosMessageName = "niryo_one_msgs/SequenceAutorunStatus";

        public bool enabled { get; set; }
        public int mode { get; set; }
        public int[] sequence_ids { get; set; }

        public SequenceAutorunStatus()
        {
            this.enabled = false;
            this.mode = 0;
            this.sequence_ids = new int[0];
        }

        public SequenceAutorunStatus(bool enabled, int mode, int[] sequence_ids)
        {
            this.enabled = enabled;
            this.mode = mode;
            this.sequence_ids = sequence_ids;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.enabled));
            listOfSerializations.Add(BitConverter.GetBytes(this.mode));
            
            listOfSerializations.Add(BitConverter.GetBytes(sequence_ids.Length));
            foreach(var entry in sequence_ids)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.enabled = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.mode = BitConverter.ToInt32(data, offset);
            offset += 4;
            
            var sequence_idsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.sequence_ids= new int[sequence_idsArrayLength];
            for(var i =0; i <sequence_idsArrayLength; i++)
            {
                this.sequence_ids[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
