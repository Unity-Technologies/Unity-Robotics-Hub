using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.NiryoOne
{
    public class DigitalIOState : Message
    {
        public const string RosMessageName = "niryo_one_msgs/DigitalIOState";

        //  GPIO pin
        public int[] pins { get; set; }
        //  PIN names seen by user to make it simpler
        public string[] names { get; set; }
        //  IN/OUT
        public int[] modes { get; set; }
        //  HIGH/LOW
        public int[] states { get; set; }

        public DigitalIOState()
        {
            this.pins = new int[0];
            this.names = new string[0];
            this.modes = new int[0];
            this.states = new int[0];
        }

        public DigitalIOState(int[] pins, string[] names, int[] modes, int[] states)
        {
            this.pins = pins;
            this.names = names;
            this.modes = modes;
            this.states = states;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(pins.Length));
            foreach(var entry in pins)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(names.Length));
            foreach(var entry in names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(modes.Length));
            foreach(var entry in modes)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(states.Length));
            foreach(var entry in states)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var pinsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.pins= new int[pinsArrayLength];
            for(var i =0; i <pinsArrayLength; i++)
            {
                this.pins[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }
            
            var namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.names= new string[namesArrayLength];
            for(var i =0; i <namesArrayLength; i++)
            {
                var namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.names[i] = DeserializeString(data, offset, namesStringBytesLength);
                offset += namesStringBytesLength;
            }
            
            var modesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.modes= new int[modesArrayLength];
            for(var i =0; i <modesArrayLength; i++)
            {
                this.modes[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }
            
            var statesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.states= new int[statesArrayLength];
            for(var i =0; i <statesArrayLength; i++)
            {
                this.states[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
