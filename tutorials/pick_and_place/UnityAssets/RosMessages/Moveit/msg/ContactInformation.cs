using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class ContactInformation : Message
    {
        public const string RosMessageName = "moveit_msgs-master/ContactInformation";

        //  Standard ROS header contains information 
        //  about the frame in which this 
        //  contact is specified
        public Header header { get; set; }
        //  Position of the contact point
        public Point position { get; set; }
        //  Normal corresponding to the contact point
        public Vector3 normal { get; set; }
        //  Depth of contact point
        public double depth { get; set; }
        //  Name of the first body that is in contact
        //  This could be a link or a namespace that represents a body
        public string contact_body_1 { get; set; }
        public uint body_type_1 { get; set; }
        //  Name of the second body that is in contact
        //  This could be a link or a namespace that represents a body
        public string contact_body_2 { get; set; }
        public uint body_type_2 { get; set; }
        public const uint ROBOT_LINK = 0;
        public const uint WORLD_OBJECT = 1;
        public const uint ROBOT_ATTACHED = 2;

        public ContactInformation()
        {
            this.header = new Header();
            this.position = new Point();
            this.normal = new Vector3();
            this.depth = 0.0;
            this.contact_body_1 = "";
            this.body_type_1 = 0;
            this.contact_body_2 = "";
            this.body_type_2 = 0;
        }

        public ContactInformation(Header header, Point position, Vector3 normal, double depth, string contact_body_1, uint body_type_1, string contact_body_2, uint body_type_2)
        {
            this.header = header;
            this.position = position;
            this.normal = normal;
            this.depth = depth;
            this.contact_body_1 = contact_body_1;
            this.body_type_1 = body_type_1;
            this.contact_body_2 = contact_body_2;
            this.body_type_2 = body_type_2;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(position.SerializationStatements());
            listOfSerializations.AddRange(normal.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.depth));
            listOfSerializations.Add(SerializeString(this.contact_body_1));
            listOfSerializations.Add(BitConverter.GetBytes(this.body_type_1));
            listOfSerializations.Add(SerializeString(this.contact_body_2));
            listOfSerializations.Add(BitConverter.GetBytes(this.body_type_2));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.position.Deserialize(data, offset);
            offset = this.normal.Deserialize(data, offset);
            this.depth = BitConverter.ToDouble(data, offset);
            offset += 8;
            var contact_body_1StringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.contact_body_1 = DeserializeString(data, offset, contact_body_1StringBytesLength);
            offset += contact_body_1StringBytesLength;
            this.body_type_1 = BitConverter.ToUInt32(data, offset);
            offset += 4;
            var contact_body_2StringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.contact_body_2 = DeserializeString(data, offset, contact_body_2StringBytesLength);
            offset += contact_body_2StringBytesLength;
            this.body_type_2 = BitConverter.ToUInt32(data, offset);
            offset += 4;

            return offset;
        }

    }
}
