using System;
using System.Collections.Generic;

namespace RosMessageGeneration
{
	public class Duration : Message
	{
        public const string RosMessageName = "std_msgs/Duration";

        public int secs { get; set; }
        public int nsecs { get; set; }

        public Duration()
        {
            secs = 0;
            nsecs = 0;
        }

        public Duration(int secs, int nsecs)
        {
        	this.secs = secs;
        	this.nsecs = nsecs;
        }

        public override List<byte[]> SerializationStatements()
        {
	        var listOfSerializations = new List<byte[]>();
	        listOfSerializations.Add(BitConverter.GetBytes(this.secs));
	        listOfSerializations.Add(BitConverter.GetBytes(this.nsecs));

	        return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
	        this.secs = BitConverter.ToInt32(data, offset);
	        offset += 4;
	        this.nsecs = BitConverter.ToInt32(data, offset);
	        offset += 4;

	        return offset;
        }
	}
}