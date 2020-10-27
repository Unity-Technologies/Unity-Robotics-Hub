using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class Constraints : Message
    {
        public const string RosMessageName = "moveit_msgs-master/Constraints";

        //  This message contains a list of motion planning constraints.
        //  All constraints must be satisfied for a goal to be considered valid
        public string name { get; set; }
        public JointConstraint[] joint_constraints { get; set; }
        public PositionConstraint[] position_constraints { get; set; }
        public OrientationConstraint[] orientation_constraints { get; set; }
        public VisibilityConstraint[] visibility_constraints { get; set; }

        public Constraints()
        {
            this.name = "";
            this.joint_constraints = new JointConstraint[0];
            this.position_constraints = new PositionConstraint[0];
            this.orientation_constraints = new OrientationConstraint[0];
            this.visibility_constraints = new VisibilityConstraint[0];
        }

        public Constraints(string name, JointConstraint[] joint_constraints, PositionConstraint[] position_constraints, OrientationConstraint[] orientation_constraints, VisibilityConstraint[] visibility_constraints)
        {
            this.name = name;
            this.joint_constraints = joint_constraints;
            this.position_constraints = position_constraints;
            this.orientation_constraints = orientation_constraints;
            this.visibility_constraints = visibility_constraints;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            
            listOfSerializations.Add(BitConverter.GetBytes(joint_constraints.Length));
            foreach(var entry in joint_constraints)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(position_constraints.Length));
            foreach(var entry in position_constraints)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(orientation_constraints.Length));
            foreach(var entry in orientation_constraints)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(visibility_constraints.Length));
            foreach(var entry in visibility_constraints)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            
            var joint_constraintsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.joint_constraints= new JointConstraint[joint_constraintsArrayLength];
            for(var i =0; i <joint_constraintsArrayLength; i++)
            {
                this.joint_constraints[i] = new JointConstraint();
                offset = this.joint_constraints[i].Deserialize(data, offset);
            }
            
            var position_constraintsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.position_constraints= new PositionConstraint[position_constraintsArrayLength];
            for(var i =0; i <position_constraintsArrayLength; i++)
            {
                this.position_constraints[i] = new PositionConstraint();
                offset = this.position_constraints[i].Deserialize(data, offset);
            }
            
            var orientation_constraintsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.orientation_constraints= new OrientationConstraint[orientation_constraintsArrayLength];
            for(var i =0; i <orientation_constraintsArrayLength; i++)
            {
                this.orientation_constraints[i] = new OrientationConstraint();
                offset = this.orientation_constraints[i].Deserialize(data, offset);
            }
            
            var visibility_constraintsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.visibility_constraints= new VisibilityConstraint[visibility_constraintsArrayLength];
            for(var i =0; i <visibility_constraintsArrayLength; i++)
            {
                this.visibility_constraints[i] = new VisibilityConstraint();
                offset = this.visibility_constraints[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
