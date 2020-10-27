using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Visualization
{
    public class MenuEntry : Message
    {
        public const string RosMessageName = "visualization_msgs/MenuEntry";

        //  MenuEntry message.
        //  Each InteractiveMarker message has an array of MenuEntry messages.
        //  A collection of MenuEntries together describe a
        //  menu/submenu/subsubmenu/etc tree, though they are stored in a flat
        //  array.  The tree structure is represented by giving each menu entry
        //  an ID number and a "parent_id" field.  Top-level entries are the
        //  ones with parent_id = 0.  Menu entries are ordered within their
        //  level the same way they are ordered in the containing array.  Parent
        //  entries must appear before their children.
        //  Example:
        //  - id = 3
        //    parent_id = 0
        //    title = "fun"
        //  - id = 2
        //    parent_id = 0
        //    title = "robot"
        //  - id = 4
        //    parent_id = 2
        //    title = "pr2"
        //  - id = 5
        //    parent_id = 2
        //    title = "turtle"
        // 
        //  Gives a menu tree like this:
        //   - fun
        //   - robot
        //     - pr2
        //     - turtle
        //  ID is a number for each menu entry.  Must be unique within the
        //  control, and should never be 0.
        public uint id { get; set; }
        //  ID of the parent of this menu entry, if it is a submenu.  If this
        //  menu entry is a top-level entry, set parent_id to 0.
        public uint parent_id { get; set; }
        //  menu / entry title
        public string title { get; set; }
        //  Arguments to command indicated by command_type (below)
        public string command { get; set; }
        //  Command_type stores the type of response desired when this menu
        //  entry is clicked.
        //  FEEDBACK: send an InteractiveMarkerFeedback message with menu_entry_id set to this entry's id.
        //  ROSRUN: execute "rosrun" with arguments given in the command field (above).
        //  ROSLAUNCH: execute "roslaunch" with arguments given in the command field (above).
        public const byte FEEDBACK = 0;
        public const byte ROSRUN = 1;
        public const byte ROSLAUNCH = 2;
        public byte command_type { get; set; }

        public MenuEntry()
        {
            this.id = 0;
            this.parent_id = 0;
            this.title = "";
            this.command = "";
            this.command_type = 0;
        }

        public MenuEntry(uint id, uint parent_id, string title, string command, byte command_type)
        {
            this.id = id;
            this.parent_id = parent_id;
            this.title = title;
            this.command = command;
            this.command_type = command_type;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.id));
            listOfSerializations.Add(BitConverter.GetBytes(this.parent_id));
            listOfSerializations.Add(SerializeString(this.title));
            listOfSerializations.Add(SerializeString(this.command));
            listOfSerializations.Add(BitConverter.GetBytes(this.command_type));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.id = BitConverter.ToUInt32(data, offset);
            offset += 4;
            this.parent_id = BitConverter.ToUInt32(data, offset);
            offset += 4;
            var titleStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.title = DeserializeString(data, offset, titleStringBytesLength);
            offset += titleStringBytesLength;
            var commandStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.command = DeserializeString(data, offset, commandStringBytesLength);
            offset += commandStringBytesLength;
            this.command_type = data[offset];;
            offset += 1;

            return offset;
        }

    }
}
