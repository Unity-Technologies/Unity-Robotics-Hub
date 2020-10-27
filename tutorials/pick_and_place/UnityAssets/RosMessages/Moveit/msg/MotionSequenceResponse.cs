using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MotionSequenceResponse : Message
    {
        public const string RosMessageName = "moveit_msgs-master/MotionSequenceResponse";

        //  An error code reflecting what went wrong
        public MoveItErrorCodes error_code { get; set; }
        //  The full starting state of the robot at the start of the sequence
        public RobotState sequence_start { get; set; }
        //  The trajectories that the planner produced for execution
        public RobotTrajectory[] planned_trajectories { get; set; }
        //  The amount of time it took to complete the motion plan
        public double planning_time { get; set; }

        public MotionSequenceResponse()
        {
            this.error_code = new MoveItErrorCodes();
            this.sequence_start = new RobotState();
            this.planned_trajectories = new RobotTrajectory[0];
            this.planning_time = 0.0;
        }

        public MotionSequenceResponse(MoveItErrorCodes error_code, RobotState sequence_start, RobotTrajectory[] planned_trajectories, double planning_time)
        {
            this.error_code = error_code;
            this.sequence_start = sequence_start;
            this.planned_trajectories = planned_trajectories;
            this.planning_time = planning_time;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(error_code.SerializationStatements());
            listOfSerializations.AddRange(sequence_start.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(planned_trajectories.Length));
            foreach(var entry in planned_trajectories)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.planning_time));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.error_code.Deserialize(data, offset);
            offset = this.sequence_start.Deserialize(data, offset);
            
            var planned_trajectoriesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.planned_trajectories= new RobotTrajectory[planned_trajectoriesArrayLength];
            for(var i =0; i <planned_trajectoriesArrayLength; i++)
            {
                this.planned_trajectories[i] = new RobotTrajectory();
                offset = this.planned_trajectories[i].Deserialize(data, offset);
            }
            this.planning_time = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
