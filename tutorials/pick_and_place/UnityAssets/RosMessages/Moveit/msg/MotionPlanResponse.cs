using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MotionPlanResponse : Message
    {
        public const string RosMessageName = "moveit_msgs-master/MotionPlanResponse";

        //  The representation of a solution to a planning problem
        //  The corresponding robot state
        public RobotState trajectory_start { get; set; }
        //  The group used for planning (usually the same as in the request)
        public string group_name { get; set; }
        //  A solution trajectory, if found
        public RobotTrajectory trajectory { get; set; }
        //  Planning time (seconds)
        public double planning_time { get; set; }
        //  Error code - encodes the overall reason for failure
        public MoveItErrorCodes error_code { get; set; }

        public MotionPlanResponse()
        {
            this.trajectory_start = new RobotState();
            this.group_name = "";
            this.trajectory = new RobotTrajectory();
            this.planning_time = 0.0;
            this.error_code = new MoveItErrorCodes();
        }

        public MotionPlanResponse(RobotState trajectory_start, string group_name, RobotTrajectory trajectory, double planning_time, MoveItErrorCodes error_code)
        {
            this.trajectory_start = trajectory_start;
            this.group_name = group_name;
            this.trajectory = trajectory;
            this.planning_time = planning_time;
            this.error_code = error_code;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(trajectory_start.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.group_name));
            listOfSerializations.AddRange(trajectory.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.planning_time));
            listOfSerializations.AddRange(error_code.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.trajectory_start.Deserialize(data, offset);
            var group_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.group_name = DeserializeString(data, offset, group_nameStringBytesLength);
            offset += group_nameStringBytesLength;
            offset = this.trajectory.Deserialize(data, offset);
            this.planning_time = BitConverter.ToDouble(data, offset);
            offset += 8;
            offset = this.error_code.Deserialize(data, offset);

            return offset;
        }

    }
}
