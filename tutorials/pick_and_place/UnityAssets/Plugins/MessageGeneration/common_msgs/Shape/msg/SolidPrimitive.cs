using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Shape
{
    public class SolidPrimitive : Message
    {
        public const string RosMessageName = "shape_msgs/SolidPrimitive";

        //  Define box, sphere, cylinder, cone 
        //  All shapes are defined to have their bounding boxes centered around 0,0,0.
        public const byte BOX = 1;
        public const byte SPHERE = 2;
        public const byte CYLINDER = 3;
        public const byte CONE = 4;
        //  The type of the shape
        public byte type { get; set; }
        //  The dimensions of the shape
        public double[] dimensions { get; set; }
        //  The meaning of the shape dimensions: each constant defines the index in the 'dimensions' array
        //  For the BOX type, the X, Y, and Z dimensions are the length of the corresponding
        //  sides of the box.
        public const byte BOX_X = 0;
        public const byte BOX_Y = 1;
        public const byte BOX_Z = 2;
        //  For the SPHERE type, only one component is used, and it gives the radius of
        //  the sphere.
        public const byte SPHERE_RADIUS = 0;
        //  For the CYLINDER and CONE types, the center line is oriented along
        //  the Z axis.  Therefore the CYLINDER_HEIGHT (CONE_HEIGHT) component
        //  of dimensions gives the height of the cylinder (cone).  The
        //  CYLINDER_RADIUS (CONE_RADIUS) component of dimensions gives the
        //  radius of the base of the cylinder (cone).  Cone and cylinder
        //  primitives are defined to be circular. The tip of the cone is
        //  pointing up, along +Z axis.
        public const byte CYLINDER_HEIGHT = 0;
        public const byte CYLINDER_RADIUS = 1;
        public const byte CONE_HEIGHT = 0;
        public const byte CONE_RADIUS = 1;

        public SolidPrimitive()
        {
            this.type = 0;
            this.dimensions = new double[0];
        }

        public SolidPrimitive(byte type, double[] dimensions)
        {
            this.type = type;
            this.dimensions = dimensions;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(BitConverter.GetBytes(this.type));
            
            listOfSerializations.Add(BitConverter.GetBytes(dimensions.Length));
            foreach(var entry in dimensions)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            this.type = data[offset];;
            offset += 1;
            
            var dimensionsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.dimensions= new double[dimensionsArrayLength];
            for(var i =0; i <dimensionsArrayLength; i++)
            {
                this.dimensions[i] = BitConverter.ToDouble(data, offset);
                offset += 8;
            }

            return offset;
        }

    }
}
