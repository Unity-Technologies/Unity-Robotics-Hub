using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Moveit
{
    public class ObjectColor : Message
    {
        public const string RosMessageName = "moveit_msgs-master/ObjectColor";

        //  The object id for which we specify color
        public string id { get; set; }
        //  The value of the color
        public ColorRGBA color { get; set; }

        public ObjectColor()
        {
            this.id = "";
            this.color = new ColorRGBA();
        }

        public ObjectColor(string id, ColorRGBA color)
        {
            this.id = id;
            this.color = color;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.id));
            listOfSerializations.AddRange(color.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.id = DeserializeString(data, offset, idStringBytesLength);
            offset += idStringBytesLength;
            offset = this.color.Deserialize(data, offset);

            return offset;
        }

    }
}
