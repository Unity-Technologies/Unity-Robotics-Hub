using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Shape;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Moveit
{
    public class BoundingVolume : Message
    {
        public const string RosMessageName = "moveit_msgs-master/BoundingVolume";

        //  Define a volume in 3D
        //  A set of solid geometric primitives that make up the volume to define (as a union)
        public SolidPrimitive[] primitives { get; set; }
        //  The poses at which the primitives are located
        public Pose[] primitive_poses { get; set; }
        //  In addition to primitives, meshes can be specified to add to the bounding volume (again, as union)
        public Mesh[] meshes { get; set; }
        //  The poses at which the meshes are located
        public Pose[] mesh_poses { get; set; }

        public BoundingVolume()
        {
            this.primitives = new SolidPrimitive[0];
            this.primitive_poses = new Pose[0];
            this.meshes = new Mesh[0];
            this.mesh_poses = new Pose[0];
        }

        public BoundingVolume(SolidPrimitive[] primitives, Pose[] primitive_poses, Mesh[] meshes, Pose[] mesh_poses)
        {
            this.primitives = primitives;
            this.primitive_poses = primitive_poses;
            this.meshes = meshes;
            this.mesh_poses = mesh_poses;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(primitives.Length));
            foreach(var entry in primitives)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(primitive_poses.Length));
            foreach(var entry in primitive_poses)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(meshes.Length));
            foreach(var entry in meshes)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(mesh_poses.Length));
            foreach(var entry in mesh_poses)
                listOfSerializations.Add(entry.Serialize());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var primitivesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.primitives= new SolidPrimitive[primitivesArrayLength];
            for(var i =0; i <primitivesArrayLength; i++)
            {
                this.primitives[i] = new SolidPrimitive();
                offset = this.primitives[i].Deserialize(data, offset);
            }
            
            var primitive_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.primitive_poses= new Pose[primitive_posesArrayLength];
            for(var i =0; i <primitive_posesArrayLength; i++)
            {
                this.primitive_poses[i] = new Pose();
                offset = this.primitive_poses[i].Deserialize(data, offset);
            }
            
            var meshesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.meshes= new Mesh[meshesArrayLength];
            for(var i =0; i <meshesArrayLength; i++)
            {
                this.meshes[i] = new Mesh();
                offset = this.meshes[i].Deserialize(data, offset);
            }
            
            var mesh_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.mesh_poses= new Pose[mesh_posesArrayLength];
            for(var i =0; i <mesh_posesArrayLength; i++)
            {
                this.mesh_poses[i] = new Pose();
                offset = this.mesh_poses[i].Deserialize(data, offset);
            }

            return offset;
        }

    }
}
