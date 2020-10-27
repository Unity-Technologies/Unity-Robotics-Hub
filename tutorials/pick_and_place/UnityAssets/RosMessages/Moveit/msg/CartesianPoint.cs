using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class CartesianPoint : Message
    {
        public const string RosMessageName = "moveit_msgs-master/CartesianPoint";

        //  This message defines a point in a cartesian trajectory
        public Pose pose { get; set; }
        public Twist velocity { get; set; }
        public Accel acceleration { get; set; }

        public CartesianPoint()
        {
            this.pose = new Pose();
            this.velocity = new Twist();
            this.acceleration = new Accel();
        }

        public CartesianPoint(Pose pose, Twist velocity, Accel acceleration)
        {
            this.pose = pose;
            this.velocity = velocity;
            this.acceleration = acceleration;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.AddRange(velocity.SerializationStatements());
            listOfSerializations.AddRange(acceleration.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.pose.Deserialize(data, offset);
            offset = this.velocity.Deserialize(data, offset);
            offset = this.acceleration.Deserialize(data, offset);

            return offset;
        }

    }
}
