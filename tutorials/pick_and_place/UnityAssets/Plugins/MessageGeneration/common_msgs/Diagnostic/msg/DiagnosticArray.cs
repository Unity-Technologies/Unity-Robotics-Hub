using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Diagnostic
{
    public class DiagnosticArray : Message
    {
        public const string RosMessageName = "diagnostic_msgs/DiagnosticArray";

        //  This message is used to send diagnostic information about the state of the robot
        public Header header { get; set; }
        // for timestamp
        public DiagnosticStatus[] status { get; set; }
        //  an array of components being reported on

        public DiagnosticArray()
        {
            this.header = new Header();
            this.status = new DiagnosticStatus[0];
        }

        public DiagnosticArray(Header header, DiagnosticStatus[] status)
        {
            this.header = header;
            this.status = status;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(status.Length));
            foreach(var entry in status)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var statusArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.status= new DiagnosticStatus[statusArrayLength];
            for(var i =0; i <statusArrayLength; i++)
            {
                this.status[i] = new DiagnosticStatus();
                offset = this.status[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
