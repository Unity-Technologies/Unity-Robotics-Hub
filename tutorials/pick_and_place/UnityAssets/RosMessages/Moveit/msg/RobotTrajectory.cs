using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Trajectory;

namespace RosMessageTypes.Moveit
{
    public class RobotTrajectory : Message
    {
        public const string RosMessageName = "moveit_msgs-master/RobotTrajectory";

        public JointTrajectory joint_trajectory { get; set; }
        public MultiDOFJointTrajectory multi_dof_joint_trajectory { get; set; }

        public RobotTrajectory()
        {
            this.joint_trajectory = new JointTrajectory();
            this.multi_dof_joint_trajectory = new MultiDOFJointTrajectory();
        }

        public RobotTrajectory(JointTrajectory joint_trajectory, MultiDOFJointTrajectory multi_dof_joint_trajectory)
        {
            this.joint_trajectory = joint_trajectory;
            this.multi_dof_joint_trajectory = multi_dof_joint_trajectory;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(joint_trajectory.SerializationStatements());
            listOfSerializations.AddRange(multi_dof_joint_trajectory.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.joint_trajectory.Deserialize(data, offset);
            offset = this.multi_dof_joint_trajectory.Deserialize(data, offset);

            return offset;
        }

    }
}
