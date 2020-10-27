using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Moveit
{
    public class CartesianTrajectory : Message
    {
        public const string RosMessageName = "moveit_msgs-master/CartesianTrajectory";

        //  This message describes the trajectory of a tracked frame in task-space
        public Header header { get; set; }
        //  The name of the Cartesian frame being tracked with respect to the base frame provided in header.frame_id
        public string tracked_frame { get; set; }
        public CartesianTrajectoryPoint[] points { get; set; }

        public CartesianTrajectory()
        {
            this.header = new Header();
            this.tracked_frame = "";
            this.points = new CartesianTrajectoryPoint[0];
        }

        public CartesianTrajectory(Header header, string tracked_frame, CartesianTrajectoryPoint[] points)
        {
            this.header = header;
            this.tracked_frame = tracked_frame;
            this.points = points;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.tracked_frame));
            
            listOfSerializations.Add(BitConverter.GetBytes(points.Length));
            foreach(var entry in points)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            var tracked_frameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.tracked_frame = DeserializeString(data, offset, tracked_frameStringBytesLength);
            offset += tracked_frameStringBytesLength;
            
            var pointsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.points= new CartesianTrajectoryPoint[pointsArrayLength];
            for(var i =0; i <pointsArrayLength; i++)
            {
                this.points[i] = new CartesianTrajectoryPoint();
                offset = this.points[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
