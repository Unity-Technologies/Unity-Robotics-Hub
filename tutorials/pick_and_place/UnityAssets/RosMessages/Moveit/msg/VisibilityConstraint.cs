using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class VisibilityConstraint : Message
    {
        public const string RosMessageName = "moveit_msgs-master/VisibilityConstraint";

        //  The constraint is useful to maintain visibility to a disc (the target) in a particular frame.
        //  This disc forms the base of a visibiliy cone whose tip is at the origin of the sensor.
        //  Maintaining visibility is done by ensuring the robot does not obstruct the visibility cone.
        //  Note:
        //  This constraint does NOT enforce minimum or maximum distances between the sensor
        //  and the target, nor does it enforce the target to be in the field of view of
        //  the sensor. A PositionConstraint can (and probably should) be used for such purposes.
        //  The radius of the disc that should be maintained visible 
        public double target_radius { get; set; }
        //  The pose of the disc; as the robot moves, the pose of the disc may change as well
        //  This can be in the frame of a particular robot link, for example
        public PoseStamped target_pose { get; set; }
        //  From the sensor origin towards the target, the disc forms a visibility cone
        //  This cone is approximated using many sides. For example, when using 4 sides, 
        //  that in fact makes the visibility region be a pyramid.
        //  This value should always be 3 or more.
        public int cone_sides { get; set; }
        //  The pose in which visibility is to be maintained.
        //  The frame id should represent the robot link to which the sensor is attached.
        //  It is assumed the sensor can look directly at the target, in any direction.
        //  This assumption is usually not true, but additional PositionConstraints
        //  can resolve this issue.
        public PoseStamped sensor_pose { get; set; }
        //  Even though the disc is maintained visible, the visibility cone can be very small
        //  because of the orientation of the disc with respect to the sensor. It is possible
        //  for example to view the disk almost from a side, in which case the visibility cone 
        //  can end up having close to 0 volume. The view angle is defined to be the angle between
        //  the normal to the visibility disc and the direction vector from the sensor origin.
        //  The value below represents the minimum desired view angle. For a perfect view,
        //  this value will be 0 (the two vectors are exact opposites). For a completely obstructed view
        //  this value will be Pi/2 (the vectors are perpendicular). This value defined below 
        //  is the maximum view angle to be maintained. This should be a value in the open interval
        //  (0, Pi/2). If 0 is set, the view angle is NOT enforced.
        public double max_view_angle { get; set; }
        //  This angle is used similarly to max_view_angle but limits the maximum angle
        //  between the sensor origin direction vector and the axis that connects the
        //  sensor origin to the target frame origin. The value is again in the range (0, Pi/2)
        //  and is NOT enforced if set to 0.
        public double max_range_angle { get; set; }
        //  The axis that is assumed to indicate the direction of view for the sensor
        //  X = 2, Y = 1, Z = 0
        public const byte SENSOR_Z = 0;
        public const byte SENSOR_Y = 1;
        public const byte SENSOR_X = 2;
        public byte sensor_view_direction { get; set; }
        //  A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
        public double weight { get; set; }

        public VisibilityConstraint()
        {
            this.target_radius = 0.0;
            this.target_pose = new PoseStamped();
            this.cone_sides = 0;
            this.sensor_pose = new PoseStamped();
            this.max_view_angle = 0.0;
            this.max_range_angle = 0.0;
            this.sensor_view_direction = 0;
            this.weight = 0.0;
        }

        public VisibilityConstraint(double target_radius, PoseStamped target_pose, int cone_sides, PoseStamped sensor_pose, double max_view_angle, double max_range_angle, byte sensor_view_direction, double weight)
        {
            this.target_radius = target_radius;
            this.target_pose = target_pose;
            this.cone_sides = cone_sides;
            this.sensor_pose = sensor_pose;
            this.max_view_angle = max_view_angle;
            this.max_range_angle = max_range_angle;
            this.sensor_view_direction = sensor_view_direction;
            this.weight = weight;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.target_radius));
            listOfSerializations.AddRange(target_pose.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.cone_sides));
            listOfSerializations.AddRange(sensor_pose.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.max_view_angle));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_range_angle));
            listOfSerializations.Add(BitConverter.GetBytes(this.sensor_view_direction));
            listOfSerializations.Add(BitConverter.GetBytes(this.weight));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.target_radius = BitConverter.ToDouble(data, offset);
            offset += 8;
            offset = this.target_pose.Deserialize(data, offset);
            this.cone_sides = BitConverter.ToInt32(data, offset);
            offset += 4;
            offset = this.sensor_pose.Deserialize(data, offset);
            this.max_view_angle = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.max_range_angle = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.sensor_view_direction = data[offset];;
            offset += 1;
            this.weight = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
