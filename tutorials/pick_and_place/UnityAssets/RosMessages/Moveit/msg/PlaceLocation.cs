using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class PlaceLocation : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PlaceLocation";

        //  A name for this grasp
        public string id { get; set; }
        //  The internal posture of the hand for the grasp
        //  positions and efforts are used
        public JointTrajectory post_place_posture { get; set; }
        //  The position of the end-effector for the grasp relative to a reference frame 
        //  (that is always specified elsewhere, not in this message)
        public PoseStamped place_pose { get; set; }
        //  The estimated probability of success for this place, or some other
        //  measure of how "good" it is.
        public double quality { get; set; }
        //  The approach motion
        public GripperTranslation pre_place_approach { get; set; }
        //  The retreat motion
        public GripperTranslation post_place_retreat { get; set; }
        //  an optional list of obstacles that we have semantic information about
        //  and that can be touched/pushed/moved in the course of grasping
        public string[] allowed_touch_objects { get; set; }

        public PlaceLocation()
        {
            this.id = "";
            this.post_place_posture = new JointTrajectory();
            this.place_pose = new PoseStamped();
            this.quality = 0.0;
            this.pre_place_approach = new GripperTranslation();
            this.post_place_retreat = new GripperTranslation();
            this.allowed_touch_objects = new string[0];
        }

        public PlaceLocation(string id, JointTrajectory post_place_posture, PoseStamped place_pose, double quality, GripperTranslation pre_place_approach, GripperTranslation post_place_retreat, string[] allowed_touch_objects)
        {
            this.id = id;
            this.post_place_posture = post_place_posture;
            this.place_pose = place_pose;
            this.quality = quality;
            this.pre_place_approach = pre_place_approach;
            this.post_place_retreat = post_place_retreat;
            this.allowed_touch_objects = allowed_touch_objects;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.id));
            listOfSerializations.AddRange(post_place_posture.SerializationStatements());
            listOfSerializations.AddRange(place_pose.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.quality));
            listOfSerializations.AddRange(pre_place_approach.SerializationStatements());
            listOfSerializations.AddRange(post_place_retreat.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(allowed_touch_objects.Length));
            foreach(var entry in allowed_touch_objects)
                listOfSerializations.Add(SerializeString(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.id = DeserializeString(data, offset, idStringBytesLength);
            offset += idStringBytesLength;
            offset = this.post_place_posture.Deserialize(data, offset);
            offset = this.place_pose.Deserialize(data, offset);
            this.quality = BitConverter.ToDouble(data, offset);
            offset += 8;
            offset = this.pre_place_approach.Deserialize(data, offset);
            offset = this.post_place_retreat.Deserialize(data, offset);
            
            var allowed_touch_objectsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.allowed_touch_objects= new string[allowed_touch_objectsArrayLength];
            for(var i =0; i <allowed_touch_objectsArrayLength; i++)
            {
                var allowed_touch_objectsStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.allowed_touch_objects[i] = DeserializeString(data, offset, allowed_touch_objectsStringBytesLength);
                offset += allowed_touch_objectsStringBytesLength;
            }

            return offset;
        }

    }
}
