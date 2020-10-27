using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class WorkspaceParameters : Message
    {
        public const string RosMessageName = "moveit_msgs-master/WorkspaceParameters";

        //  This message contains a set of parameters useful in
        //  setting up the volume (a box) in which the robot is allowed to move.
        //  This is useful only when planning for mobile parts of 
        //  the robot as well.
        //  Define the frame of reference for the box corners
        public Header header { get; set; }
        //  The minumum corner of the box, with respect to the robot starting pose
        public Vector3 min_corner { get; set; }
        //  The maximum corner of the box, with respect to the robot starting pose
        public Vector3 max_corner { get; set; }

        public WorkspaceParameters()
        {
            this.header = new Header();
            this.min_corner = new Vector3();
            this.max_corner = new Vector3();
        }

        public WorkspaceParameters(Header header, Vector3 min_corner, Vector3 max_corner)
        {
            this.header = header;
            this.min_corner = min_corner;
            this.max_corner = max_corner;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(min_corner.SerializationStatements());
            listOfSerializations.AddRange(max_corner.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.min_corner.Deserialize(data, offset);
            offset = this.max_corner.Deserialize(data, offset);

            return offset;
        }

    }
}
