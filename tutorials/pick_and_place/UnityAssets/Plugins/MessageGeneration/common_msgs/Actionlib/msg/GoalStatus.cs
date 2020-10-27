using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Actionlib
{
    public class GoalStatus : Message
    {
        public const string RosMessageName = "actionlib_msgs/GoalStatus";

        public GoalID goal_id { get; set; }
        public byte status { get; set; }
        public const byte PENDING = 0; //  The goal has yet to be processed by the action server
        public const byte ACTIVE = 1; //  The goal is currently being processed by the action server
        public const byte PREEMPTED = 2; //  The goal received a cancel request after it started executing
        //    and has since completed its execution (Terminal State)
        public const byte SUCCEEDED = 3; //  The goal was achieved successfully by the action server (Terminal State)
        public const byte ABORTED = 4; //  The goal was aborted during execution by the action server due
        //     to some failure (Terminal State)
        public const byte REJECTED = 5; //  The goal was rejected by the action server without being processed,
        //     because the goal was unattainable or invalid (Terminal State)
        public const byte PREEMPTING = 6; //  The goal received a cancel request after it started executing
        //     and has not yet completed execution
        public const byte RECALLING = 7; //  The goal received a cancel request before it started executing,
        //     but the action server has not yet confirmed that the goal is canceled
        public const byte RECALLED = 8; //  The goal received a cancel request before it started executing
        //     and was successfully cancelled (Terminal State)
        public const byte LOST = 9; //  An action client can determine that a goal is LOST. This should not be
        //     sent over the wire by an action server
        // Allow for the user to associate a string with GoalStatus for debugging
        public string text { get; set; }

        public GoalStatus()
        {
            this.goal_id = new GoalID();
            this.status = 0;
            this.text = "";
        }

        public GoalStatus(GoalID goal_id, byte status, string text)
        {
            this.goal_id = goal_id;
            this.status = status;
            this.text = text;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(goal_id.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.status));
            listOfSerializations.Add(SerializeString(this.text));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.goal_id.Deserialize(data, offset);
            this.status = data[offset];;
            offset += 1;
            var textStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.text = DeserializeString(data, offset, textStringBytesLength);
            offset += textStringBytesLength;

            return offset;
        }

    }
}
