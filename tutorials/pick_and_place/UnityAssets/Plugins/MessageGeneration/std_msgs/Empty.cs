using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Std
{
    public class Empty : Message
    {
        public const string RosMessageName = "std_msgs/Empty";


        public Empty()
        {
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {

            return offset;
        }

    }
}
