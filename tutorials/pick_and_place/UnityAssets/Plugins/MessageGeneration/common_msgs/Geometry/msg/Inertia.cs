using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Geometry
{
    public class Inertia : Message
    {
        public const string RosMessageName = "geometry_msgs/Inertia";

        //  Mass [kg]
        public double m { get; set; }
        //  Center of mass [m]
        public Vector3 com { get; set; }
        //  Inertia Tensor [kg-m^2]
        //      | ixx ixy ixz |
        //  I = | ixy iyy iyz |
        //      | ixz iyz izz |
        public double ixx { get; set; }
        public double ixy { get; set; }
        public double ixz { get; set; }
        public double iyy { get; set; }
        public double iyz { get; set; }
        public double izz { get; set; }

        public Inertia()
        {
            this.m = 0.0;
            this.com = new Vector3();
            this.ixx = 0.0;
            this.ixy = 0.0;
            this.ixz = 0.0;
            this.iyy = 0.0;
            this.iyz = 0.0;
            this.izz = 0.0;
        }

        public Inertia(double m, Vector3 com, double ixx, double ixy, double ixz, double iyy, double iyz, double izz)
        {
            this.m = m;
            this.com = com;
            this.ixx = ixx;
            this.ixy = ixy;
            this.ixz = ixz;
            this.iyy = iyy;
            this.iyz = iyz;
            this.izz = izz;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.m));
            listOfSerializations.AddRange(com.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.ixx));
            listOfSerializations.Add(BitConverter.GetBytes(this.ixy));
            listOfSerializations.Add(BitConverter.GetBytes(this.ixz));
            listOfSerializations.Add(BitConverter.GetBytes(this.iyy));
            listOfSerializations.Add(BitConverter.GetBytes(this.iyz));
            listOfSerializations.Add(BitConverter.GetBytes(this.izz));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.m = BitConverter.ToDouble(data, offset);
            offset += 8;
            offset = this.com.Deserialize(data, offset);
            this.ixx = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.ixy = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.ixz = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.iyy = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.iyz = BitConverter.ToDouble(data, offset);
            offset += 8;
            this.izz = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
