using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Sensor
{
    public class LaserEcho : Message
    {
        public const string RosMessageName = "sensor_msgs/LaserEcho";

        //  This message is a submessage of MultiEchoLaserScan and is not intended
        //  to be used separately.
        public float[] echoes { get; set; }
        //  Multiple values of ranges or intensities.
        //  Each array represents data from the same angle increment.

        public LaserEcho()
        {
            this.echoes = new float[0];
        }

        public LaserEcho(float[] echoes)
        {
            this.echoes = echoes;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(echoes.Length));
            foreach(var entry in echoes)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var echoesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.echoes= new float[echoesArrayLength];
            for(var i =0; i <echoesArrayLength; i++)
            {
                this.echoes[i] = BitConverter.ToSingle(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
