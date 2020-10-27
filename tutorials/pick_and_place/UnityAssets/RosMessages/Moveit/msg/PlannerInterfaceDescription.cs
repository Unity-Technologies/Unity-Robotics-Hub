using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class PlannerInterfaceDescription : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PlannerInterfaceDescription";

        //  The name of the planner interface
        public string name { get; set; }
        //  The names of the planner ids within the interface
        public string[] planner_ids { get; set; }

        public PlannerInterfaceDescription()
        {
            this.name = "";
            this.planner_ids = new string[0];
        }

        public PlannerInterfaceDescription(string name, string[] planner_ids)
        {
            this.name = name;
            this.planner_ids = planner_ids;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            
            listOfSerializations.Add(BitConverter.GetBytes(planner_ids.Length));
            foreach(var entry in planner_ids)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            
            var planner_idsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.planner_ids= new string[planner_idsArrayLength];
            for(var i =0; i <planner_idsArrayLength; i++)
            {
                var planner_idsStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.planner_ids[i] = DeserializeString(data, offset, planner_idsStringBytesLength);
                offset += planner_idsStringBytesLength;
            }

            return offset;
        }

    }
}
