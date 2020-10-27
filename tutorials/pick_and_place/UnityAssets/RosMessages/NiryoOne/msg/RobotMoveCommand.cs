using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.NiryoOne;

namespace RosMessageTypes.NiryoOne
{
    public class RobotMoveCommand : Message
    {
        public const string RosMessageName = "niryo_one_msgs/RobotMoveCommand";

        public int cmd_type { get; set; }
        public double[] joints { get; set; }
        public Point position { get; set; }
        public RPY rpy { get; set; }
        public ShiftPose shift { get; set; }
        public TrajectoryPlan Trajectory { get; set; }
        public Pose pose_quat { get; set; }
        public string saved_position_name { get; set; }
        public int saved_trajectory_id { get; set; }
        public ToolCommand tool_cmd { get; set; }
        //  In the future, allow a tool command to be launched at the same time as an Arm command
        //  3 choices : arm only, arm + tool, tool only
        //  bool use_tool 

        public RobotMoveCommand()
        {
            this.cmd_type = 0;
            this.joints = new double[0];
            this.position = new Point();
            this.rpy = new RPY();
            this.shift = new ShiftPose();
            this.Trajectory = new TrajectoryPlan();
            this.pose_quat = new Pose();
            this.saved_position_name = "";
            this.saved_trajectory_id = 0;
            this.tool_cmd = new ToolCommand();
        }

        public RobotMoveCommand(int cmd_type, double[] joints, Point position, RPY rpy, ShiftPose shift, TrajectoryPlan Trajectory, Pose pose_quat, string saved_position_name, int saved_trajectory_id, ToolCommand tool_cmd)
        {
            this.cmd_type = cmd_type;
            this.joints = joints;
            this.position = position;
            this.rpy = rpy;
            this.shift = shift;
            this.Trajectory = Trajectory;
            this.pose_quat = pose_quat;
            this.saved_position_name = saved_position_name;
            this.saved_trajectory_id = saved_trajectory_id;
            this.tool_cmd = tool_cmd;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.cmd_type));
            
            listOfSerializations.Add(BitConverter.GetBytes(joints.Length));
            foreach(var entry in joints)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            listOfSerializations.AddRange(position.SerializationStatements());
            listOfSerializations.AddRange(rpy.SerializationStatements());
            listOfSerializations.AddRange(shift.SerializationStatements());
            listOfSerializations.AddRange(Trajectory.SerializationStatements());
            listOfSerializations.AddRange(pose_quat.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.saved_position_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.saved_trajectory_id));
            listOfSerializations.AddRange(tool_cmd.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.cmd_type = BitConverter.ToInt32(data, offset);
            offset += 4;
            
            var jointsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.joints= new double[jointsArrayLength];
            for(var i =0; i <jointsArrayLength; i++)
            {
                this.joints[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            offset = this.position.Deserialize(data, offset);
            offset = this.rpy.Deserialize(data, offset);
            offset = this.shift.Deserialize(data, offset);
            offset = this.Trajectory.Deserialize(data, offset);
            offset = this.pose_quat.Deserialize(data, offset);
            var saved_position_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.saved_position_name = DeserializeString(data, offset, saved_position_nameStringBytesLength);
            offset += saved_position_nameStringBytesLength;
            this.saved_trajectory_id = BitConverter.ToInt32(data, offset);
            offset += 4;
            offset = this.tool_cmd.Deserialize(data, offset);

            return offset;
        }

    }
}
