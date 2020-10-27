using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;

namespace RosMessageTypes.Sensor
{
    public class Joy : Message
    {
        public const string RosMessageName = "sensor_msgs/Joy";

        //  Reports the state of a joysticks axes and buttons.
        public Header header { get; set; }
        //  timestamp in the header is the time the data is received from the joystick
        public float[] axes { get; set; }
        //  the axes measurements from a joystick
        public int[] buttons { get; set; }
        //  the buttons measurements from a joystick 

        public Joy()
        {
            this.header = new Header();
            this.axes = new float[0];
            this.buttons = new int[0];
        }

        public Joy(Header header, float[] axes, int[] buttons)
        {
            this.header = header;
            this.axes = axes;
            this.buttons = buttons;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(axes.Length));
            foreach(var entry in axes)
                listOfSerializations.Add(BitConverter.GetBytes(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(buttons.Length));
            foreach(var entry in buttons)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            
            var axesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.axes= new float[axesArrayLength];
            for(var i =0; i <axesArrayLength; i++)
            {
                this.axes[i] = BitConverter.ToSingle(data, offset);
                offset += 4;
            }
            
            var buttonsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.buttons= new int[buttonsArrayLength];
            for(var i =0; i <buttonsArrayLength; i++)
            {
                this.buttons[i] = BitConverter.ToInt32(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
