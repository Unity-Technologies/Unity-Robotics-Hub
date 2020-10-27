using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class ToolCommand : Message
    {
        public const string RosMessageName = "niryo_one_msgs/ToolCommand";

        public byte tool_id { get; set; }
        public byte cmd_type { get; set; }
        //  if gripper close
        public ushort gripper_close_speed { get; set; }
        //  if gripper open
        public ushort gripper_open_speed { get; set; }
        //  if vacuum pump or electromagnet grove
        public bool activate { get; set; }
        //  if tool is set by digital outputs (electromagnet)
        public byte gpio { get; set; }

        public ToolCommand()
        {
            this.tool_id = 0;
            this.cmd_type = 0;
            this.gripper_close_speed = 0;
            this.gripper_open_speed = 0;
            this.activate = false;
            this.gpio = 0;
        }

        public ToolCommand(byte tool_id, byte cmd_type, ushort gripper_close_speed, ushort gripper_open_speed, bool activate, byte gpio)
        {
            this.tool_id = tool_id;
            this.cmd_type = cmd_type;
            this.gripper_close_speed = gripper_close_speed;
            this.gripper_open_speed = gripper_open_speed;
            this.activate = activate;
            this.gpio = gpio;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.tool_id));
            listOfSerializations.Add(BitConverter.GetBytes(this.cmd_type));
            listOfSerializations.Add(BitConverter.GetBytes(this.gripper_close_speed));
            listOfSerializations.Add(BitConverter.GetBytes(this.gripper_open_speed));
            listOfSerializations.Add(BitConverter.GetBytes(this.activate));
            listOfSerializations.Add(BitConverter.GetBytes(this.gpio));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.tool_id = data[offset];;
            offset += 1;
            this.cmd_type = data[offset];;
            offset += 1;
            this.gripper_close_speed = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.gripper_open_speed = BitConverter.ToUInt16(data, offset);
            offset += 2;
            this.activate = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.gpio = data[offset];;
            offset += 1;

            return offset;
        }

    }
}
