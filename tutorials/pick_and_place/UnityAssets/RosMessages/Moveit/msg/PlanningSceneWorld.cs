using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Octomap;

namespace RosMessageTypes.Moveit
{
    public class PlanningSceneWorld : Message
    {
        public const string RosMessageName = "moveit_msgs-master/PlanningSceneWorld";

        //  collision objects
        public CollisionObject[] collision_objects { get; set; }
        //  The octomap that represents additional collision data
        public OctomapWithPose octomap { get; set; }

        public PlanningSceneWorld()
        {
            this.collision_objects = new CollisionObject[0];
            this.octomap = new OctomapWithPose();
        }

        public PlanningSceneWorld(CollisionObject[] collision_objects, OctomapWithPose octomap)
        {
            this.collision_objects = collision_objects;
            this.octomap = octomap;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            
            listOfSerializations.Add(BitConverter.GetBytes(collision_objects.Length));
            foreach(var entry in collision_objects)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(octomap.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            
            var collision_objectsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.collision_objects= new CollisionObject[collision_objectsArrayLength];
            for(var i =0; i <collision_objectsArrayLength; i++)
            {
                this.collision_objects[i] = new CollisionObject();
                offset = this.collision_objects[i].Deserialize(data, offset);
            }
            offset = this.octomap.Deserialize(data, offset);

            return offset;
        }

    }
}
