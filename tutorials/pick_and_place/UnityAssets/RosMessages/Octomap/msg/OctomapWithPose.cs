using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.Octomap;

namespace RosMessageTypes.Octomap
{
    public class OctomapWithPose : Message
    {
        public const string RosMessageName = "octomap_msgs-melodic-devel/OctomapWithPose";

        //  A 3D map in binary format, as Octree
        public Header header { get; set; }
        //  The pose of the octree with respect to the header frame 
        public Pose origin { get; set; }
        //  The actual octree msg
        public Octomap octomap { get; set; }

        public OctomapWithPose()
        {
            this.header = new Header();
            this.origin = new Pose();
            this.octomap = new Octomap();
        }

        public OctomapWithPose(Header header, Pose origin, Octomap octomap)
        {
            this.header = header;
            this.origin = origin;
            this.octomap = octomap;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(origin.SerializationStatements());
            listOfSerializations.AddRange(octomap.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.origin.Deserialize(data, offset);
            offset = this.octomap.Deserialize(data, offset);

            return offset;
        }

    }
}
