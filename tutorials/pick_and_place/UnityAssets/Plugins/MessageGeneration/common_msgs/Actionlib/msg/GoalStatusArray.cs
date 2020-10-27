using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Actionlib
{
    public class GoalStatusArray : Message
    {
        public const string RosMessageName = "actionlib_msgs/GoalStatusArray";

        //  Stores the statuses for goals that are currently being tracked
        //  by an action server
        public Header header { get; set; }
        public GoalStatus[] status_list { get; set; }

        public GoalStatusArray()
        {
            this.header = new Header();
            this.status_list = new GoalStatus[0];
        }

        public GoalStatusArray(Header header, GoalStatus[] status_list)
        {
            this.header = header;
            this.status_list = status_list;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(status_list.Length));
            foreach(var entry in status_list)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var status_listArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.status_list= new GoalStatus[status_listArrayLength];
            for(var i =0; i <status_listArrayLength; i++)
            {
                this.status_list[i] = new GoalStatus();
                offset = this.status_list[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
