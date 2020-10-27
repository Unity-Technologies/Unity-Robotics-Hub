using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Shape
{
    public class Plane : Message
    {
        public const string RosMessageName = "shape_msgs/Plane";

        //  Representation of a plane, using the plane equation ax + by + cz + d = 0
        //  a := coef[0]
        //  b := coef[1]
        //  c := coef[2]
        //  d := coef[3]
        public double[] coef { get; set; }

        public Plane()
        {
            this.coef = new double[4];
        }

        public Plane(double[] coef)
        {
            this.coef = coef;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(coef.Length));
            foreach(var entry in coef)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var coefArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.coef= new double[coefArrayLength];
            for(var i =0; i <coefArrayLength; i++)
            {
                this.coef[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }

            return offset;
        }

    }
}
