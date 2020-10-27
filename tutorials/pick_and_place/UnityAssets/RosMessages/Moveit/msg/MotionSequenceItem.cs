using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MotionSequenceItem : Message
    {
        public const string RosMessageName = "moveit_msgs-master/MotionSequenceItem";

        //  The plan request for this item.
        //  It is the planning request for this segment of the sequence, as if it were a solitary motion.
        public MotionPlanRequest req { get; set; }
        //  To blend between sequence items, the motion may be smoothed using a circular motion.
        //  The blend radius of the circle between this and the next command, where 0 means no blending.
        public double blend_radius { get; set; }

        public MotionSequenceItem()
        {
            this.req = new MotionPlanRequest();
            this.blend_radius = 0.0;
        }

        public MotionSequenceItem(MotionPlanRequest req, double blend_radius)
        {
            this.req = req;
            this.blend_radius = blend_radius;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(req.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.blend_radius));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.req.Deserialize(data, offset);
            this.blend_radius = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
