using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class GripperTranslation : Message
    {
        public const string RosMessageName = "moveit_msgs-master/GripperTranslation";

        //  defines a translation for the gripper, used in pickup or place tasks
        //  for example for lifting an object off a table or approaching the table for placing
        //  the direction of the translation
        public Vector3Stamped direction { get; set; }
        //  the desired translation distance
        public float desired_distance { get; set; }
        //  the min distance that must be considered feasible before the
        //  grasp is even attempted
        public float min_distance { get; set; }

        public GripperTranslation()
        {
            this.direction = new Vector3Stamped();
            this.desired_distance = 0.0f;
            this.min_distance = 0.0f;
        }

        public GripperTranslation(Vector3Stamped direction, float desired_distance, float min_distance)
        {
            this.direction = direction;
            this.desired_distance = desired_distance;
            this.min_distance = min_distance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(direction.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.desired_distance));
            listOfSerializations.Add(BitConverter.GetBytes(this.min_distance));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.direction.Deserialize(data, offset);
            this.desired_distance = BitConverter.ToSingle(data, offset);
            offset += 4;
            this.min_distance = BitConverter.ToSingle(data, offset);
            offset += 4;

            return offset;
        }

    }
}
