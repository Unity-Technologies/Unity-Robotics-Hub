using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Trajectory;
using RosMessageTypes.Moveit;

namespace RosMessageTypes.Moveit
{
    public class GenericTrajectory : Message
    {
        public const string RosMessageName = "moveit_msgs-master/GenericTrajectory";

        //  A generic trajectory message that may either encode a joint- or cartesian-space trajectory, or both
        //  Trajectories encoded in this message are considered semantically equivalent
        public Header header { get; set; }
        public JointTrajectory[] joint_trajectory { get; set; }
        public CartesianTrajectory[] cartesian_trajectory { get; set; }

        public GenericTrajectory()
        {
            this.header = new Header();
            this.joint_trajectory = new JointTrajectory[0];
            this.cartesian_trajectory = new CartesianTrajectory[0];
        }

        public GenericTrajectory(Header header, JointTrajectory[] joint_trajectory, CartesianTrajectory[] cartesian_trajectory)
        {
            this.header = header;
            this.joint_trajectory = joint_trajectory;
            this.cartesian_trajectory = cartesian_trajectory;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(joint_trajectory.Length));
            foreach(var entry in joint_trajectory)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(cartesian_trajectory.Length));
            foreach(var entry in cartesian_trajectory)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var joint_trajectoryArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.joint_trajectory= new JointTrajectory[joint_trajectoryArrayLength];
            for(var i =0; i <joint_trajectoryArrayLength; i++)
            {
                this.joint_trajectory[i] = new JointTrajectory();
                offset = this.joint_trajectory[i].Deserialize(data, offset);
            }
            
            var cartesian_trajectoryArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.cartesian_trajectory= new CartesianTrajectory[cartesian_trajectoryArrayLength];
            for(var i =0; i <cartesian_trajectoryArrayLength; i++)
            {
                this.cartesian_trajectory[i] = new CartesianTrajectory();
                offset = this.cartesian_trajectory[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
