using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Sensor;

namespace RosMessageTypes.Moveit
{
    public class RobotState : Message
    {
        public const string RosMessageName = "moveit_msgs-master/RobotState";

        //  This message contains information about the robot state, i.e. the positions of its joints and links
        public JointState joint_state { get; set; }
        //  Joints that may have multiple DOF are specified here
        public MultiDOFJointState multi_dof_joint_state { get; set; }
        //  Attached collision objects (attached to some link on the robot)
        public AttachedCollisionObject[] attached_collision_objects { get; set; }
        //  Flag indicating whether this scene is to be interpreted as a diff with respect to some other scene
        //  This is mostly important for handling the attached bodies (whether or not to clear the attached bodies
        //  of a moveit::core::RobotState before updating it with this message)
        public bool is_diff { get; set; }

        public RobotState()
        {
            this.joint_state = new JointState();
            this.multi_dof_joint_state = new MultiDOFJointState();
            this.attached_collision_objects = new AttachedCollisionObject[0];
            this.is_diff = false;
        }

        public RobotState(JointState joint_state, MultiDOFJointState multi_dof_joint_state, AttachedCollisionObject[] attached_collision_objects, bool is_diff)
        {
            this.joint_state = joint_state;
            this.multi_dof_joint_state = multi_dof_joint_state;
            this.attached_collision_objects = attached_collision_objects;
            this.is_diff = is_diff;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(joint_state.SerializationStatements());
            listOfSerializations.AddRange(multi_dof_joint_state.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(attached_collision_objects.Length));
            foreach(var entry in attached_collision_objects)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.is_diff));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.joint_state.Deserialize(data, offset);
            offset = this.multi_dof_joint_state.Deserialize(data, offset);
            
            var attached_collision_objectsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.attached_collision_objects= new AttachedCollisionObject[attached_collision_objectsArrayLength];
            for(var i =0; i <attached_collision_objectsArrayLength; i++)
            {
                this.attached_collision_objects[i] = new AttachedCollisionObject();
                offset = this.attached_collision_objects[i].Deserialize(data, offset);
            }
            this.is_diff = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
