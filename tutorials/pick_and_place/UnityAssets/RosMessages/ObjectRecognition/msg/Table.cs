using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.ObjectRecognition
{
    public class Table : Message
    {
        public const string RosMessageName = "object_recognition_msgs-master/Table";

        //  Informs that a planar table has been detected at a given location
        public Header header { get; set; }
        //  The pose gives you the transform that take you to the coordinate system
        //  of the table, with the origin somewhere in the table plane and the 
        //  z axis normal to the plane
        public Pose pose { get; set; }
        //  There is no guarantee that the table does NOT extend further than the
        //  convex hull; this is just as far as we've observed it.
        //  The origin of the table coordinate system is inside the convex hull
        //  Set of points forming the convex hull of the table
        public Point[] convex_hull { get; set; }

        public Table()
        {
            this.header = new Header();
            this.pose = new Pose();
            this.convex_hull = new Point[0];
        }

        public Table(Header header, Pose pose, Point[] convex_hull)
        {
            this.header = header;
            this.pose = pose;
            this.convex_hull = convex_hull;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(pose.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(convex_hull.Length));
            foreach(var entry in convex_hull)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.pose.Deserialize(data, offset);
            
            var convex_hullArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.convex_hull= new Point[convex_hullArrayLength];
            for(var i =0; i <convex_hullArrayLength; i++)
            {
                this.convex_hull[i] = new Point();
                offset = this.convex_hull[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
