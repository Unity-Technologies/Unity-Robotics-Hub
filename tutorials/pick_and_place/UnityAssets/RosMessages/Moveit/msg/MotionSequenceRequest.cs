using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MotionSequenceRequest : Message
    {
        public const string RosMessageName = "moveit_msgs-master/MotionSequenceRequest";

        //  List of motion planning request with a blend_radius for each.
        //  In the response of the planner all of these will be executable as one sequence.
        public MotionSequenceItem[] items { get; set; }

        public MotionSequenceRequest()
        {
            this.items = new MotionSequenceItem[0];
        }

        public MotionSequenceRequest(MotionSequenceItem[] items)
        {
            this.items = items;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(items.Length));
            foreach(var entry in items)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var itemsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.items= new MotionSequenceItem[itemsArrayLength];
            for(var i =0; i <itemsArrayLength; i++)
            {
                this.items[i] = new MotionSequenceItem();
                offset = this.items[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
