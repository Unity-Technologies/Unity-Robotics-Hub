using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MotionPlanDetailedResponse : Message
    {
        public const string RosMessageName = "moveit_msgs-master/MotionPlanDetailedResponse";

        //  The representation of a solution to a planning problem, including intermediate data
        //  The starting state considered for the robot solution path
        public RobotState trajectory_start { get; set; }
        //  The group used for planning (usually the same as in the request)
        public string group_name { get; set; }
        //  Multiple solution paths are reported, each reflecting intermediate steps in the trajectory processing
        //  The list of reported trajectories
        public RobotTrajectory[] trajectory { get; set; }
        //  Description of the reported trajectories (name of processing step)
        public string[] description { get; set; }
        //  The amount of time spent computing a particular step in motion plan computation 
        public double[] processing_time { get; set; }
        //  Status at the end of this plan
        public MoveItErrorCodes error_code { get; set; }

        public MotionPlanDetailedResponse()
        {
            this.trajectory_start = new RobotState();
            this.group_name = "";
            this.trajectory = new RobotTrajectory[0];
            this.description = new string[0];
            this.processing_time = new double[0];
            this.error_code = new MoveItErrorCodes();
        }

        public MotionPlanDetailedResponse(RobotState trajectory_start, string group_name, RobotTrajectory[] trajectory, string[] description, double[] processing_time, MoveItErrorCodes error_code)
        {
            this.trajectory_start = trajectory_start;
            this.group_name = group_name;
            this.trajectory = trajectory;
            this.description = description;
            this.processing_time = processing_time;
            this.error_code = error_code;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(trajectory_start.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.group_name));
            
            listOfSerializations.Add(BitConverter.GetBytes(trajectory.Length));
            foreach(var entry in trajectory)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(description.Length));
            foreach(var entry in description)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(processing_time.Length));
            foreach(var entry in processing_time)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
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
            
            var trajectoryArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.trajectory= new RobotTrajectory[trajectoryArrayLength];
            for(var i =0; i <trajectoryArrayLength; i++)
            {
                this.trajectory[i] = new RobotTrajectory();
                offset = this.trajectory[i].Deserialize(data, offset);
            }
            
            var descriptionArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.description= new string[descriptionArrayLength];
            for(var i =0; i <descriptionArrayLength; i++)
            {
                var descriptionStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.description[i] = DeserializeString(data, offset, descriptionStringBytesLength);
                offset += descriptionStringBytesLength;
            }
            
            var processing_timeArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.processing_time= new double[processing_timeArrayLength];
            for(var i =0; i <processing_timeArrayLength; i++)
            {
                this.processing_time[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            offset = this.error_code.Deserialize(data, offset);

            return offset;
        }

    }
}
