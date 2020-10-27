using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class CostSource : Message
    {
        public const string RosMessageName = "moveit_msgs-master/CostSource";

        //  The density of the cost source
        public double cost_density { get; set; }
        //  The volume of the cost source is represented as an
        //  axis-aligned bounding box (AABB)
        //  The AABB is specified by two of its opposite corners
        public Vector3 aabb_min { get; set; }
        public Vector3 aabb_max { get; set; }

        public CostSource()
        {
            this.cost_density = 0.0;
            this.aabb_min = new Vector3();
            this.aabb_max = new Vector3();
        }

        public CostSource(double cost_density, Vector3 aabb_min, Vector3 aabb_max)
        {
            this.cost_density = cost_density;
            this.aabb_min = aabb_min;
            this.aabb_max = aabb_max;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.cost_density));
            listOfSerializations.AddRange(aabb_min.SerializationStatements());
            listOfSerializations.AddRange(aabb_max.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.cost_density = BitConverter.ToDouble(data, offset);
            offset += 8;
            offset = this.aabb_min.Deserialize(data, offset);
            offset = this.aabb_max.Deserialize(data, offset);

            return offset;
        }

    }
}
