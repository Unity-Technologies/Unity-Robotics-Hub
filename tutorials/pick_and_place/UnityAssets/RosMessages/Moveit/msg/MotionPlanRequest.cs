using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class MotionPlanRequest : Message
    {
        public const string RosMessageName = "moveit_msgs-master/MotionPlanRequest";

        //  This service contains the definition for a request to the motion
        //  planner and the output it provides
        //  Parameters for the workspace that the planner should work inside
        public WorkspaceParameters workspace_parameters { get; set; }
        //  Starting state updates. If certain joints should be considered
        //  at positions other than the current ones, these positions should
        //  be set here
        public RobotState start_state { get; set; }
        //  The possible goal states for the model to plan for. Each element of
        //  the array defines a goal region. The goal is achieved
        //  if the constraints for a particular region are satisfied
        public Constraints[] goal_constraints { get; set; }
        //  No state at any point along the path in the produced motion plan will violate these constraints (this applies to all points, not just waypoints)
        public Constraints path_constraints { get; set; }
        //  The constraints the resulting trajectory must satisfy
        public TrajectoryConstraints trajectory_constraints { get; set; }
        //  A set of trajectories that may be used as reference or initial trajectories for (typically optimization-based) planners
        //  These trajectories do not override start_state or goal_constraints
        public GenericTrajectory[] reference_trajectories { get; set; }
        //  The name of the motion planner to use. If no name is specified,
        //  a default motion planner will be used
        public string planner_id { get; set; }
        //  The name of the group of joints on which this planner is operating
        public string group_name { get; set; }
        //  The number of times this plan is to be computed. Shortest solution
        //  will be reported.
        public int num_planning_attempts { get; set; }
        //  The maximum amount of time the motion planner is allowed to plan for (in seconds)
        public double allowed_planning_time { get; set; }
        //  Scaling factors for optionally reducing the maximum joint velocities and
        //  accelerations.  Allowed values are in (0,1].  The maximum joint velocity and
        //  acceleration specified in the robot model are multiplied by thier respective
        //  factors.  If either are outside their valid ranges (importantly, this
        //  includes being set to 0.0), the factor is set to the default value of 1.0
        //  internally (i.e., maximum joint velocity or maximum joint acceleration).
        public double max_velocity_scaling_factor { get; set; }
        public double max_acceleration_scaling_factor { get; set; }
        //  Maximum cartesian speed for the given end effector.
        //  If max_cartesian_speed <= 0 the trajectory is not modified.
        //  These fields require the following planning request adapter: default_planner_request_adapters/SetMaxCartesianEndEffectorSpeed
        public string cartesian_speed_end_effector_link { get; set; }
        public double max_cartesian_speed { get; set; }
        //  m/s

        public MotionPlanRequest()
        {
            this.workspace_parameters = new WorkspaceParameters();
            this.start_state = new RobotState();
            this.goal_constraints = new Constraints[0];
            this.path_constraints = new Constraints();
            this.trajectory_constraints = new TrajectoryConstraints();
            this.reference_trajectories = new GenericTrajectory[0];
            this.planner_id = "";
            this.group_name = "";
            this.num_planning_attempts = 0;
            this.allowed_planning_time = 0.0;
            this.max_velocity_scaling_factor = 0.0;
            this.max_acceleration_scaling_factor = 0.0;
            this.cartesian_speed_end_effector_link = "";
            this.max_cartesian_speed = 0.0;
        }

        public MotionPlanRequest(WorkspaceParameters workspace_parameters, RobotState start_state, Constraints[] goal_constraints, Constraints path_constraints, TrajectoryConstraints trajectory_constraints, GenericTrajectory[] reference_trajectories, string planner_id, string group_name, int num_planning_attempts, double allowed_planning_time, double max_velocity_scaling_factor, double max_acceleration_scaling_factor, string cartesian_speed_end_effector_link, double max_cartesian_speed)
        {
            this.workspace_parameters = workspace_parameters;
            this.start_state = start_state;
            this.goal_constraints = goal_constraints;
            this.path_constraints = path_constraints;
            this.trajectory_constraints = trajectory_constraints;
            this.reference_trajectories = reference_trajectories;
            this.planner_id = planner_id;
            this.group_name = group_name;
            this.num_planning_attempts = num_planning_attempts;
            this.allowed_planning_time = allowed_planning_time;
            this.max_velocity_scaling_factor = max_velocity_scaling_factor;
            this.max_acceleration_scaling_factor = max_acceleration_scaling_factor;
            this.cartesian_speed_end_effector_link = cartesian_speed_end_effector_link;
            this.max_cartesian_speed = max_cartesian_speed;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(workspace_parameters.SerializationStatements());
            listOfSerializations.AddRange(start_state.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(goal_constraints.Length));
            foreach(var entry in goal_constraints)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(path_constraints.SerializationStatements());
            listOfSerializations.AddRange(trajectory_constraints.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(reference_trajectories.Length));
            foreach(var entry in reference_trajectories)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(SerializeString(this.planner_id));
            listOfSerializations.Add(SerializeString(this.group_name));
            listOfSerializations.Add(BitConverter.GetBytes(this.num_planning_attempts));
            listOfSerializations.Add(BitConverter.GetBytes(this.allowed_planning_time));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_velocity_scaling_factor));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_acceleration_scaling_factor));
            listOfSerializations.Add(SerializeString(this.cartesian_speed_end_effector_link));
            listOfSerializations.Add(BitConverter.GetBytes(this.max_cartesian_speed));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.workspace_parameters.Deserialize(data, offset);
            offset = this.start_state.Deserialize(data, offset);
            
            var goal_constraintsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.goal_constraints= new Constraints[goal_constraintsArrayLength];
            for(var i =0; i <goal_constraintsArrayLength; i++)
            {
                this.goal_constraints[i] = new Constraints();
                offset = this.goal_constraints[i].Deserialize(data, offset);
            }
            offset = this.path_constraints.Deserialize(data, offset);
            offset = this.trajectory_constraints.Deserialize(data, offset);
            
            var reference_trajectoriesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.reference_trajectories= new GenericTrajectory[reference_trajectoriesArrayLength];
            for(var i =0; i <reference_trajectoriesArrayLength; i++)
            {
                this.reference_trajectories[i] = new GenericTrajectory();
                offset = this.reference_trajectories[i].Deserialize(data, offset);
            }
            var planner_idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.planner_id = DeserializeString(data, offset, planner_idStringBytesLength);
            offset += planner_idStringBytesLength;
            var group_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.group_name = DeserializeString(data, offset, group_nameStringBytesLength);
            offset += group_nameStringBytesLength;
            this.num_planning_attempts = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.allowed_planning_time = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.max_velocity_scaling_factor = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.max_acceleration_scaling_factor = BitConverter.ToDouble(data, offset);
            offset += 8;
            var cartesian_speed_end_effector_linkStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.cartesian_speed_end_effector_link = DeserializeString(data, offset, cartesian_speed_end_effector_linkStringBytesLength);
            offset += cartesian_speed_end_effector_linkStringBytesLength;
            this.max_cartesian_speed = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
