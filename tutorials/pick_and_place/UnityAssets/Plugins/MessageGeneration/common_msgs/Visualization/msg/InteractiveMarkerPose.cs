using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Visualization
{
    public class InteractiveMarkerPose : Message
    {
        public const string RosMessageName = "visualization_msgs/InteractiveMarkerPose";

        //  Time/frame info.
        public Header header { get; set; }
        //  Initial pose. Also, defines the pivot point for rotations.
        public Pose pose { get; set; }
        //  Identifying string. Must be globally unique in
        //  the topic that this message is sent through.
        public string name { get; set; }

        public InteractiveMarkerPose()
        {
            this.header = new Header();
            this.pose = new Pose();
            this.name = "";
        }

        public InteractiveMarkerPose(Header header, Pose pose, string name)
        {
            this.header = header;
            this.pose = pose;
            this.name = name;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.name));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.pose.Deserialize(data, offset);
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;

            return offset;
        }

    }
}
