using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;
using RosMessageTypes.Std;

namespace RosMessageTypes.Trajectory
{
    public class MultiDOFJointTrajectoryPoint : Message
    {
        public const string RosMessageName = "trajectory_msgs/MultiDOFJointTrajectoryPoint";

        //  Each multi-dof joint can specify a transform (up to 6 DOF)
        public Transform[] transforms { get; set; }
        //  There can be a velocity specified for the origin of the joint 
        public Twist[] velocities { get; set; }
        //  There can be an acceleration specified for the origin of the joint 
        public Twist[] accelerations { get; set; }
        public Duration time_from_start { get; set; }

        public MultiDOFJointTrajectoryPoint()
        {
            this.transforms = new Transform[0];
            this.velocities = new Twist[0];
            this.accelerations = new Twist[0];
            this.time_from_start = new Duration();
        }

        public MultiDOFJointTrajectoryPoint(Transform[] transforms, Twist[] velocities, Twist[] accelerations, Duration time_from_start)
        {
            this.transforms = transforms;
            this.velocities = velocities;
            this.accelerations = accelerations;
            this.time_from_start = time_from_start;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(transforms.Length));
            foreach(var entry in transforms)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(velocities.Length));
            foreach(var entry in velocities)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(accelerations.Length));
            foreach(var entry in accelerations)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(time_from_start.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var transformsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.transforms= new Transform[transformsArrayLength];
            for(var i =0; i <transformsArrayLength; i++)
            {
                this.transforms[i] = new Transform();
                offset = this.transforms[i].Deserialize(data, offset);
            }
            
            var velocitiesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.velocities= new Twist[velocitiesArrayLength];
            for(var i =0; i <velocitiesArrayLength; i++)
            {
                this.velocities[i] = new Twist();
                offset = this.velocities[i].Deserialize(data, offset);
            }
            
            var accelerationsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.accelerations= new Twist[accelerationsArrayLength];
            for(var i =0; i <accelerationsArrayLength; i++)
            {
                this.accelerations[i] = new Twist();
                offset = this.accelerations[i].Deserialize(data, offset);
            }
            offset = this.time_from_start.Deserialize(data, offset);

            return offset;
        }

    }
}
