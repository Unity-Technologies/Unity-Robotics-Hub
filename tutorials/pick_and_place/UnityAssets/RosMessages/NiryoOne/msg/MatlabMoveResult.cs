using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class MatlabMoveResult : Message
    {
        public const string RosMessageName = "niryo_one_msgs/MatlabMoveResult";

        public int status { get; set; }
        public string message { get; set; }

        public MatlabMoveResult()
        {
            this.status = 0;
            this.message = "";
        }

        public MatlabMoveResult(int status, string message)
        {
            this.status = status;
            this.message = message;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.status));
            listOfSerializations.Add(SerializeString(this.message));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.status = BitConverter.ToInt32(data, offset);
            offset += 4;
            var messageStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.message = DeserializeString(data, offset, messageStringBytesLength);
            offset += messageStringBytesLength;

            return offset;
        }

    }
}
