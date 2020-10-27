using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;
using RosMessageTypes.ObjectRecognition;
using RosMessageTypes.Shape;

namespace RosMessageTypes.Moveit
{
    public class CollisionObject : Message
    {
        public const string RosMessageName = "moveit_msgs-master/CollisionObject";

        //  A header, used for interpreting the poses
        public Header header { get; set; }
        //  DISCLAIMER: This field is not in use yet and all other poses
        //  are still interpreted in the header frame.
        //  https://github.com/ros-planning/moveit/pull/2037
        //  implements the actual logic for this field.
        //  ---
        //  The object's pose relative to the header frame.
        //  The shapes and subframe poses are defined relative to this pose.
        public Pose pose { get; set; }
        //  The id of the object (name used in MoveIt)
        public string id { get; set; }
        //  The object type in a database of known objects
        public ObjectType type { get; set; }
        //  The collision geometries associated with the object.
        //  Their poses are with respect to the object's pose
        //  Solid geometric primitives
        public SolidPrimitive[] primitives { get; set; }
        public Pose[] primitive_poses { get; set; }
        //  Meshes
        public Mesh[] meshes { get; set; }
        public Pose[] mesh_poses { get; set; }
        //  Bounding planes (equation is specified, but the plane can be oriented using an additional pose)
        public Plane[] planes { get; set; }
        public Pose[] plane_poses { get; set; }
        //  Named subframes on the object. Use these to define points of interest on the object that you want
        //  to plan with (e.g. "tip", "spout", "handle"). The id of the object will be prepended to the subframe.
        //  If an object with the id "screwdriver" and a subframe "tip" is in the scene, you can use the frame
        //  "screwdriver/tip" for planning.
        //  The length of the subframe_names and subframe_poses has to be identical.
        public string[] subframe_names { get; set; }
        public Pose[] subframe_poses { get; set; }
        //  Adds the object to the planning scene. If the object previously existed, it is replaced.
        public const sbyte ADD = 0;
        //  Removes the object from the environment entirely (everything that matches the specified id)
        public const sbyte REMOVE = 1;
        //  Append to an object that already exists in the planning scene. If the object does not exist, it is added.
        public const sbyte APPEND = 2;
        //  If an object already exists in the scene, new poses can be sent (the geometry arrays must be left empty)
        //  if solely moving the object is desired
        public const sbyte MOVE = 3;
        //  Operation to be performed
        public sbyte operation { get; set; }

        public CollisionObject()
        {
            this.header = new Header();
            this.pose = new Pose();
            this.id = "";
            this.type = new ObjectType();
            this.primitives = new SolidPrimitive[0];
            this.primitive_poses = new Pose[0];
            this.meshes = new Mesh[0];
            this.mesh_poses = new Pose[0];
            this.planes = new Plane[0];
            this.plane_poses = new Pose[0];
            this.subframe_names = new string[0];
            this.subframe_poses = new Pose[0];
            this.operation = 0;
        }

        public CollisionObject(Header header, Pose pose, string id, ObjectType type, SolidPrimitive[] primitives, Pose[] primitive_poses, Mesh[] meshes, Pose[] mesh_poses, Plane[] planes, Pose[] plane_poses, string[] subframe_names, Pose[] subframe_poses, sbyte operation)
        {
            this.header = header;
            this.pose = pose;
            this.id = id;
            this.type = type;
            this.primitives = primitives;
            this.primitive_poses = primitive_poses;
            this.meshes = meshes;
            this.mesh_poses = mesh_poses;
            this.planes = planes;
            this.plane_poses = plane_poses;
            this.subframe_names = subframe_names;
            this.subframe_poses = subframe_poses;
            this.operation = operation;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.id));
            listOfSerializations.AddRange(type.SerializationStatements());
            
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
            
            listOfSerializations.Add(BitConverter.GetBytes(planes.Length));
            foreach(var entry in planes)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(plane_poses.Length));
            foreach(var entry in plane_poses)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(subframe_names.Length));
            foreach(var entry in subframe_names)
                listOfSerializations.Add(SerializeString(entry));
            
            listOfSerializations.Add(BitConverter.GetBytes(subframe_poses.Length));
            foreach(var entry in subframe_poses)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(BitConverter.GetBytes(this.operation));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.pose.Deserialize(data, offset);
            var idStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.id = DeserializeString(data, offset, idStringBytesLength);
            offset += idStringBytesLength;
            offset = this.type.Deserialize(data, offset);
            
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
            
            var planesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.planes= new Plane[planesArrayLength];
            for(var i =0; i <planesArrayLength; i++)
            {
                this.planes[i] = new Plane();
                offset = this.planes[i].Deserialize(data, offset);
            }
            
            var plane_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.plane_poses= new Pose[plane_posesArrayLength];
            for(var i =0; i <plane_posesArrayLength; i++)
            {
                this.plane_poses[i] = new Pose();
                offset = this.plane_poses[i].Deserialize(data, offset);
            }
            
            var subframe_namesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.subframe_names= new string[subframe_namesArrayLength];
            for(var i =0; i <subframe_namesArrayLength; i++)
            {
                var subframe_namesStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.subframe_names[i] = DeserializeString(data, offset, subframe_namesStringBytesLength);
                offset += subframe_namesStringBytesLength;
            }
            
            var subframe_posesArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.subframe_poses= new Pose[subframe_posesArrayLength];
            for(var i =0; i <subframe_posesArrayLength; i++)
            {
                this.subframe_poses[i] = new Pose();
                offset = this.subframe_poses[i].Deserialize(data, offset);
            }
            this.operation = (sbyte)data[offset];;
            offset += 1;

            return offset;
        }

    }
}
