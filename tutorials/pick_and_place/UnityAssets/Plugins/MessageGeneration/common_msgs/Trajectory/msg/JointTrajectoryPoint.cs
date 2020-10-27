using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Trajectory
{
    public class JointTrajectoryPoint : Message
    {
        public const string RosMessageName = "trajectory_msgs/JointTrajectoryPoint";

        //  Each trajectory point specifies either positions[, velocities[, accelerations]]
        //  or positions[, effort] for the trajectory to be executed.
        //  All specified values are in the same order as the joint names in JointTrajectory.msg
        public double[] positions { get; set; }
        public double[] velocities { get; set; }
        public double[] accelerations { get; set; }
        public double[] effort { get; set; }
        public Duration time_from_start { get; set; }

        public JointTrajectoryPoint()
        {
            this.positions = new double[0];
            this.velocities = new double[0];
            this.accelerations = new double[0];
            this.effort = new double[0];
            this.time_from_start = new Duration();
        }

        public JointTrajectoryPoint(double[] positions, double[] velocities, double[] accelerations, double[] effort, Duration time_from_start)
        {
            this.positions = positions;
            this.velocities = velocities;
            this.accelerations = accelerations;
            this.effort = effort;
            this.time_from_start = time_from_start;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(positions.Length));
            foreach(var entry in positions)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(velocities.Length));
            foreach(var entry in velocities)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(accelerations.Length));
            foreach(var entry in accelerations)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(effort.Length));
            foreach(var entry in effort)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            listOfSerializations.AddRange(time_from_start.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var positionsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.positions= new double[positionsArrayLength];
            for(var i =0; i <positionsArrayLength; i++)
            {
                this.positions[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var velocitiesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.velocities= new double[velocitiesArrayLength];
            for(var i =0; i <velocitiesArrayLength; i++)
            {
                this.velocities[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var accelerationsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.accelerations= new double[accelerationsArrayLength];
            for(var i =0; i <accelerationsArrayLength; i++)
            {
                this.accelerations[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            
            var effortArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.effort= new double[effortArrayLength];
            for(var i =0; i <effortArrayLength; i++)
            {
                this.effort[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }
            offset = this.time_from_start.Deserialize(data, offset);

            return offset;
        }

    }
}
