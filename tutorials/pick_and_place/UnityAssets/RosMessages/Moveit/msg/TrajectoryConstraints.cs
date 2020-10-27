using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class TrajectoryConstraints : Message
    {
        public const string RosMessageName = "moveit_msgs-master/TrajectoryConstraints";

        //  The array of constraints to consider along the trajectory
        public Constraints[] constraints { get; set; }

        public TrajectoryConstraints()
        {
            this.constraints = new Constraints[0];
        }

        public TrajectoryConstraints(Constraints[] constraints)
        {
            this.constraints = constraints;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(constraints.Length));
            foreach(var entry in constraints)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var constraintsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.constraints= new Constraints[constraintsArrayLength];
            for(var i =0; i <constraintsArrayLength; i++)
            {
                this.constraints[i] = new Constraints();
                offset = this.constraints[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
