using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Shape
{
    public class Mesh : Message
    {
        public const string RosMessageName = "shape_msgs/Mesh";

        //  Definition of a mesh
        //  list of triangles; the index values refer to positions in vertices[]
        public MeshTriangle[] triangles { get; set; }
        //  the actual vertices that make up the mesh
        public Point[] vertices { get; set; }

        public Mesh()
        {
            this.triangles = new MeshTriangle[0];
            this.vertices = new Point[0];
        }

        public Mesh(MeshTriangle[] triangles, Point[] vertices)
        {
            this.triangles = triangles;
            this.vertices = vertices;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(triangles.Length));
            foreach(var entry in triangles)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(vertices.Length));
            foreach(var entry in vertices)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var trianglesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.triangles= new MeshTriangle[trianglesArrayLength];
            for(var i =0; i <trianglesArrayLength; i++)
            {
                this.triangles[i] = new MeshTriangle();
                offset = this.triangles[i].Deserialize(data, offset);
            }
            
            var verticesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.vertices= new Point[verticesArrayLength];
            for(var i =0; i <verticesArrayLength; i++)
            {
                this.vertices[i] = new Point();
                offset = this.vertices[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
