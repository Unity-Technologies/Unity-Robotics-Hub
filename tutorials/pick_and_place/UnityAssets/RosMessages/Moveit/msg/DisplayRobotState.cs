using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class DisplayRobotState : Message
    {
        public const string RosMessageName = "moveit_msgs-master/DisplayRobotState";

        //  The robot state to display
        public RobotState state { get; set; }
        //  Optionally, various links can be highlighted
        public ObjectColor[] highlight_links { get; set; }
        //  If true, suppress the display in visualizations (like rviz)
        public bool hide { get; set; }

        public DisplayRobotState()
        {
            this.state = new RobotState();
            this.highlight_links = new ObjectColor[0];
            this.hide = false;
        }

        public DisplayRobotState(RobotState state, ObjectColor[] highlight_links, bool hide)
        {
            this.state = state;
            this.highlight_links = highlight_links;
            this.hide = hide;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(state.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(highlight_links.Length));
            foreach(var entry in highlight_links)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.hide));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.state.Deserialize(data, offset);
            
            var highlight_linksArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.highlight_links= new ObjectColor[highlight_linksArrayLength];
            for(var i =0; i <highlight_linksArrayLength; i++)
            {
                this.highlight_links[i] = new ObjectColor();
                offset = this.highlight_links[i].Deserialize(data, offset);
            }
            this.hide = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
