using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Actionlib
{
    public class GoalID : Message
    {
        public const string RosMessageName = "actionlib_msgs/GoalID";

        //  The stamp should store the time at which this goal was requested.
        //  It is used by an action server when it tries to preempt all
        //  goals that were requested before a certain time
        public Time stamp { get; set; }
        //  The id provides a way to associate feedback and
        //  result message with specific goal requests. The id
        //  specified must be unique.
        public string id { get; set; }

        public GoalID()
        {
            this.stamp = new Time();
            this.id = "";
        }

        public GoalID(Time stamp, string id)
        {
            this.stamp = stamp;
            this.id = id;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(stamp.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.id));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.stamp.Deserialize(data, offset);
            var idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.id = DeserializeString(data, offset, idStringBytesLength);
            offset += idStringBytesLength;

            return offset;
        }

    }
}
