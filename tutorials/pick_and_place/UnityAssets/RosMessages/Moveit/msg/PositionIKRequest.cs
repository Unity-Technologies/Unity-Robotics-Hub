using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Moveit;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

namespace RosMessageTypes.Moveit
{
    public class PositionIKRequest : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PositionIKRequest";

        //  A Position IK request message
        //  The name of the group which will be used to compute IK
        //  e.g. "right_arm", or "arms" - see IK specification for multiple-groups below
        //  Information from the SRDF will be used to automatically determine which link 
        //  to solve IK for, unless ik_link_name is also specified
        public string group_name { get; set; }
        //  A RobotState consisting of hint/seed positions for the IK computation and positions 
        //  for all the other joints in the robot. Additional state information provided here is 
        //  used to specify starting positions for other joints/links on the robot.  
        //  This state MUST contain state for all joints to be used by the IK solver
        //  to compute IK. The list of joints that the IK solver deals with can be 
        //  found using the SRDF for the corresponding group
        public RobotState robot_state { get; set; }
        //  A set of constraints that the IK must obey; by default, this set of constraints is empty
        public Constraints constraints { get; set; }
        //  Find an IK solution that avoids collisions. By default, this is false
        public bool avoid_collisions { get; set; }
        //  (OPTIONAL) The name of the link for which we are computing IK
        //  If not specified, the link name will be inferred from a combination 
        //  of the group name and the SRDF. If any values are specified for ik_link_names,
        //  this value is ignored
        public string ik_link_name { get; set; }
        //  The stamped pose of the link, when the IK solver computes the joint values
        //  for all the joints in a group. This value is ignored if pose_stamped_vector
        //  has any elements specified.
        public PoseStamped pose_stamped { get; set; }
        //  Multi-group parameters
        //  (OPTIONAL) The names of the links for which we are computing IK
        //  If not specified, the link name will be inferred from a combination 
        //  of the group name and the SRDF
        public string[] ik_link_names { get; set; }
        //  (OPTIONAL) The (stamped) poses of the links we are computing IK for (when a group has more than one end effector)
        //  e.g. The "arms" group might consist of both the "right_arm" and the "left_arm"
        //  The order of the groups referred to is the same as the order setup in the SRDF
        public PoseStamped[] pose_stamped_vector { get; set; }
        //  Maximum allowed time for IK calculation
        public Duration timeout { get; set; }

        public PositionIKRequest()
        {
            this.group_name = "";
            this.robot_state = new RobotState();
            this.constraints = new Constraints();
            this.avoid_collisions = false;
            this.ik_link_name = "";
            this.pose_stamped = new PoseStamped();
            this.ik_link_names = new string[0];
            this.pose_stamped_vector = new PoseStamped[0];
            this.timeout = new Duration();
        }

        public PositionIKRequest(string group_name, RobotState robot_state, Constraints constraints, bool avoid_collisions, string ik_link_name, PoseStamped pose_stamped, string[] ik_link_names, PoseStamped[] pose_stamped_vector, Duration timeout)
        {
            this.group_name = group_name;
            this.robot_state = robot_state;
            this.constraints = constraints;
            this.avoid_collisions = avoid_collisions;
            this.ik_link_name = ik_link_name;
            this.pose_stamped = pose_stamped;
            this.ik_link_names = ik_link_names;
            this.pose_stamped_vector = pose_stamped_vector;
            this.timeout = timeout;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.group_name));
            listOfSerializations.AddRange(robot_state.SerializationStatements());
            listOfSerializations.AddRange(constraints.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.avoid_collisions));
            listOfSerializations.Add(SerializeString(this.ik_link_name));
            listOfSerializations.AddRange(pose_stamped.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(ik_link_names.Length));
            foreach(var entry in ik_link_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(pose_stamped_vector.Length));
            foreach(var entry in pose_stamped_vector)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(timeout.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var group_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.group_name = DeserializeString(data, offset, group_nameStringBytesLength);
            offset += group_nameStringBytesLength;
            offset = this.robot_state.Deserialize(data, offset);
            offset = this.constraints.Deserialize(data, offset);
            this.avoid_collisions = BitConverter.ToBoolean(data, offset);
            offset += 1;
            var ik_link_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.ik_link_name = DeserializeString(data, offset, ik_link_nameStringBytesLength);
            offset += ik_link_nameStringBytesLength;
            offset = this.pose_stamped.Deserialize(data, offset);
            
            var ik_link_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.ik_link_names= new string[ik_link_namesArrayLength];
            for(var i =0; i <ik_link_namesArrayLength; i++)
            {
                var ik_link_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.ik_link_names[i] = DeserializeString(data, offset, ik_link_namesStringBytesLength);
                offset += ik_link_namesStringBytesLength;
            }
            
            var pose_stamped_vectorArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.pose_stamped_vector= new PoseStamped[pose_stamped_vectorArrayLength];
            for(var i =0; i <pose_stamped_vectorArrayLength; i++)
            {
                this.pose_stamped_vector[i] = new PoseStamped();
                offset = this.pose_stamped_vector[i].Deserialize(data, offset);
            }
            offset = this.timeout.Deserialize(data, offset);

            return offset;
        }

    }
}
