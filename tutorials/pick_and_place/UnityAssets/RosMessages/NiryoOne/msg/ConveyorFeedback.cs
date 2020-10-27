using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class ConveyorFeedback : Message
    {
        public const string RosMessageName = "niryo_one_msgs/ConveyorFeedback";

        // Conveyor id ( either 6 or 7) 
        public byte conveyor_id { get; set; }
        // Conveyor Connection state ( if it is enabled) 
        public bool connection_state { get; set; }
        //  Conveyor Controls state : ON or OFF
        public bool running { get; set; }
        //  Conveyor Speed ( 1-> 100 %)
        public short speed { get; set; }
        //  Conveyor direction ( backward or forward)
        public sbyte direction { get; set; }

        public ConveyorFeedback()
        {
            this.conveyor_id = 0;
            this.connection_state = false;
            this.running = false;
            this.speed = 0;
            this.direction = 0;
        }

        public ConveyorFeedback(byte conveyor_id, bool connection_state, bool running, short speed, sbyte direction)
        {
            this.conveyor_id = conveyor_id;
            this.connection_state = connection_state;
            this.running = running;
            this.speed = speed;
            this.direction = direction;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.conveyor_id));
            listOfSerializations.Add(BitConverter.GetBytes(this.connection_state));
            listOfSerializations.Add(BitConverter.GetBytes(this.running));
            listOfSerializations.Add(BitConverter.GetBytes(this.speed));
            listOfSerializations.Add(BitConverter.GetBytes(this.direction));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.conveyor_id = data[offset];;
            offset += 1;
            this.connection_state = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.running = BitConverter.ToBoolean(data, offset);
            offset += 1;
            this.speed = BitConverter.ToInt16(data, offset);
            offset += 2;
            this.direction = (sbyte)data[offset];;
            offset += 1;

            return offset;
        }

    }
}
