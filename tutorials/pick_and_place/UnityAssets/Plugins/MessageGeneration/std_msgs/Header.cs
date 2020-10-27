using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Std
{
    public class Header : Message
    {
        public const string RosMessageName = "std_msgs/Header";

        //  Standard metadata for higher-level stamped data types.
        //  This is generally used to communicate timestamped data 
        //  in a particular coordinate frame.
        //  
        //  sequence ID: consecutively increasing ID 
        public uint seq { get; set; }
        // Two-integer timestamp that is expressed as:
        //  * stamp.sec: seconds (stamp_secs) since epoch (in Python the variable is called 'secs')
        //  * stamp.nsec: nanoseconds since stamp_secs (in Python the variable is called 'nsecs')
        //  time-handling sugar is provided by the client library
        public Time stamp { get; set; }
        // Frame this data is associated with
        public string frame_id { get; set; }

        public Header()
        {
            this.seq = 0;
            this.stamp = new Time();
            this.frame_id = "";
        }

        public Header(uint seq, Time stamp, string frame_id)
        {
            this.seq = seq;
            this.stamp = stamp;
            this.frame_id = frame_id;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.seq));
            listOfSerializations.AddRange(stamp.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.frame_id));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.seq = BitConverter.ToUInt32(data, offset);
            offset += 4;
            offset = this.stamp.Deserialize(data, offset);
            var stringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.frame_id = DeserializeString(data, offset, stringBytesLength);
            offset += stringBytesLength;

            return offset;
        }

    }
}
