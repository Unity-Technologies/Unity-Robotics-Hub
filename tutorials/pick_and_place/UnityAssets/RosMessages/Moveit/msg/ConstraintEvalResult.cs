using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Moveit
{
    public class ConstraintEvalResult : Message
    {
        public const string RosMessageName = "moveit_msgs-master/ConstraintEvalResult";

        //  This message contains result from constraint evaluation
        //  result specifies the result of constraint evaluation 
        //  (true indicates state satisfies constraint, false indicates state violates constraint)
        //  if false, distance specifies a measure of the distance of the state from the constraint
        //  if true, distance is set to zero
        public bool result { get; set; }
        public double distance { get; set; }

        public ConstraintEvalResult()
        {
            this.result = false;
            this.distance = 0.0;
        }

        public ConstraintEvalResult(bool result, double distance)
        {
            this.result = result;
            this.distance = distance;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.result));
            listOfSerializations.Add(BitConverter.GetBytes(this.distance));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.result = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.distance = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
