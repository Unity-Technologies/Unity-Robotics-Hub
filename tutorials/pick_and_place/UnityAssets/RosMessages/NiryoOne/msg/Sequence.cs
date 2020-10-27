using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class Sequence : Message
    {
        public const string RosMessageName = "niryo_one_msgs/Sequence";

        public int id { get; set; }
        public string name { get; set; }
        public string description { get; set; }
        public int created { get; set; }
        public int updated { get; set; }
        public string blockly_xml { get; set; }
        public string python_code { get; set; }

        public Sequence()
        {
            this.id = 0;
            this.name = "";
            this.description = "";
            this.created = 0;
            this.updated = 0;
            this.blockly_xml = "";
            this.python_code = "";
        }

        public Sequence(int id, string name, string description, int created, int updated, string blockly_xml, string python_code)
        {
            this.id = id;
            this.name = name;
            this.description = description;
            this.created = created;
            this.updated = updated;
            this.blockly_xml = blockly_xml;
            this.python_code = python_code;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.id));
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.Add(SerializeString(this.description));
            listOfSerializations.Add(BitConverter.GetBytes(this.created));
            listOfSerializations.Add(BitConverter.GetBytes(this.updated));
            listOfSerializations.Add(SerializeString(this.blockly_xml));
            listOfSerializations.Add(SerializeString(this.python_code));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.id = BitConverter.ToInt32(data, offset);
            offset += 4;
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            var descriptionStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.description = DeserializeString(data, offset, descriptionStringBytesLength);
            offset += descriptionStringBytesLength;
            this.created = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.updated = BitConverter.ToInt32(data, offset);
            offset += 4;
            var blockly_xmlStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.blockly_xml = DeserializeString(data, offset, blockly_xmlStringBytesLength);
            offset += blockly_xmlStringBytesLength;
            var python_codeStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.python_code = DeserializeString(data, offset, python_codeStringBytesLength);
            offset += python_codeStringBytesLength;

            return offset;
        }

    }
}
