using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class Grasp : Message
    {
        public const string RosMessageName = "moveit_msgs-master/Grasp";

        //  This message contains a description of a grasp that would be used
        //  with a particular end-effector to grasp an object, including how to
        //  approach it, grip it, etc.  This message does not contain any
        //  information about a "grasp point" (a position ON the object).
        //  Whatever generates this message should have already combined
        //  information about grasp points with information about the geometry
        //  of the end-effector to compute the grasp_pose in this message.
        //  A name for this grasp
        public string id { get; set; }
        //  The internal posture of the hand for the pre-grasp
        //  only positions are used
        public JointTrajectory pre_grasp_posture { get; set; }
        //  The internal posture of the hand for the grasp
        //  positions and efforts are used
        public JointTrajectory grasp_posture { get; set; }
        //  The position of the end-effector for the grasp.  This is the pose of
        //  the "parent_link" of the end-effector, not actually the pose of any
        //  link *in* the end-effector.  Typically this would be the pose of the
        //  most distal wrist link before the hand (end-effector) links began.
        public PoseStamped grasp_pose { get; set; }
        //  The estimated probability of success for this grasp, or some other
        //  measure of how "good" it is.
        public double grasp_quality { get; set; }
        //  The approach direction to take before picking an object
        public GripperTranslation pre_grasp_approach { get; set; }
        //  The retreat direction to take after a grasp has been completed (object is attached)
        public GripperTranslation post_grasp_retreat { get; set; }
        //  The retreat motion to perform when releasing the object; this information
        //  is not necessary for the grasp itself, but when releasing the object,
        //  the information will be necessary. The grasp used to perform a pickup
        //  is returned as part of the result, so this information is available for 
        //  later use.
        public GripperTranslation post_place_retreat { get; set; }
        //  the maximum contact force to use while grasping (<=0 to disable)
        public float max_contact_force { get; set; }
        //  an optional list of obstacles that we have semantic information about
        //  and that can be touched/pushed/moved in the course of grasping
        public string[] allowed_touch_objects { get; set; }

        public Grasp()
        {
            this.id = "";
            this.pre_grasp_posture = new JointTrajectory();
            this.grasp_posture = new JointTrajectory();
            this.grasp_pose = new PoseStamped();
            this.grasp_quality = 0.0;
            this.pre_grasp_approach = new GripperTranslation();
            this.post_grasp_retreat = new GripperTranslation();
            this.post_place_retreat = new GripperTranslation();
            this.max_contact_force = 0.0f;
            this.allowed_touch_objects = new string[0];
        }

        public Grasp(string id, JointTrajectory pre_grasp_posture, JointTrajectory grasp_posture, PoseStamped grasp_pose, double grasp_quality, GripperTranslation pre_grasp_approach, GripperTranslation post_grasp_retreat, GripperTranslation post_place_retreat, float max_contact_force, string[] allowed_touch_objects)
        {
            this.id = id;
            this.pre_grasp_posture = pre_grasp_posture;
            this.grasp_posture = grasp_posture;
            this.grasp_pose = grasp_pose;
            this.grasp_quality = grasp_quality;
            this.pre_grasp_approach = pre_grasp_approach;
            this.post_grasp_retreat = post_grasp_retreat;
            this.post_place_retreat = post_place_retreat;
            this.max_contact_force = max_contact_force;
            this.allowed_touch_objects = allowed_touch_objects;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.id));
            listOfSerializations.AddRange(pre_grasp_posture.SerializationStatements());
            listOfSerializations.AddRange(grasp_posture.SerializationStatements());
            listOfSerializations.AddRange(grasp_pose.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.grasp_quality));
            listOfSerializations.AddRange(pre_grasp_approach.SerializationStatements());
            listOfSerializations.AddRange(post_grasp_retreat.SerializationStatements());
            listOfSerializations.AddRange(post_place_retreat.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.max_contact_force));
            
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
            offset = this.pre_grasp_posture.Deserialize(data, offset);
            offset = this.grasp_posture.Deserialize(data, offset);
            offset = this.grasp_pose.Deserialize(data, offset);
            this.grasp_quality = BitConverter.ToDouble(data, offset);
            offset += 8;
            offset = this.pre_grasp_approach.Deserialize(data, offset);
            offset = this.post_grasp_retreat.Deserialize(data, offset);
            offset = this.post_place_retreat.Deserialize(data, offset);
            this.max_contact_force = BitConverter.ToSingle(data, offset);
            offset += 4;
            
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
