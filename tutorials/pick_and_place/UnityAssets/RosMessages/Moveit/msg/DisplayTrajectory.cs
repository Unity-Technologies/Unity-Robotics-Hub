using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class DisplayTrajectory : Message
    {
        public const string RosMessageName = "moveit_msgs-master/DisplayTrajectory";

        //  The model id for which this path has been generated
        public string model_id { get; set; }
        //  The representation of the path contains position values for all the joints that are moving along the path; a sequence of trajectories may be specified
        public RobotTrajectory[] trajectory { get; set; }
        //  The robot state is used to obtain positions for all/some of the joints of the robot. 
        //  It is used by the path display node to determine the positions of the joints that are not specified in the joint path message above. 
        //  If the robot state message contains joint position information for joints that are also mentioned in the joint path message, the positions in the joint path message will overwrite the positions specified in the robot state message. 
        public RobotState trajectory_start { get; set; }

        public DisplayTrajectory()
        {
            this.model_id = "";
            this.trajectory = new RobotTrajectory[0];
            this.trajectory_start = new RobotState();
        }

        public DisplayTrajectory(string model_id, RobotTrajectory[] trajectory, RobotState trajectory_start)
        {
            this.model_id = model_id;
            this.trajectory = trajectory;
            this.trajectory_start = trajectory_start;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.model_id));
            
            listOfSerializations.Add(BitConverter.GetBytes(trajectory.Length));
            foreach(var entry in trajectory)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(trajectory_start.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var model_idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.model_id = DeserializeString(data, offset, model_idStringBytesLength);
            offset += model_idStringBytesLength;
            
            var trajectoryArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.trajectory= new RobotTrajectory[trajectoryArrayLength];
            for(var i =0; i <trajectoryArrayLength; i++)
            {
                this.trajectory[i] = new RobotTrajectory();
                offset = this.trajectory[i].Deserialize(data, offset);
            }
            offset = this.trajectory_start.Deserialize(data, offset);

            return offset;
        }

    }
}
