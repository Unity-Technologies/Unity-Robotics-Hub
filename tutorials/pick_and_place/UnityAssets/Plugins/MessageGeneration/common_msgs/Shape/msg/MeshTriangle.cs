using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Shape
{
    public class MeshTriangle : Message
    {
        public const string RosMessageName = "shape_msgs/MeshTriangle";

        //  Definition of a triangle's vertices
        public uint[] vertex_indices { get; set; }

        public MeshTriangle()
        {
            this.vertex_indices = new uint[3];
        }

        public MeshTriangle(uint[] vertex_indices)
        {
            this.vertex_indices = vertex_indices;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(vertex_indices.Length));
            foreach(var entry in vertex_indices)
                listOfSerializations.Add(BitConverter.GetBytes(entry));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var vertex_indicesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.vertex_indices= new uint[vertex_indicesArrayLength];
            for(var i =0; i <vertex_indicesArrayLength; i++)
            {
                this.vertex_indices[i] = BitConverter.ToUInt32(data, offset);
                offset += 4;
            }

            return offset;
        }

    }
}
