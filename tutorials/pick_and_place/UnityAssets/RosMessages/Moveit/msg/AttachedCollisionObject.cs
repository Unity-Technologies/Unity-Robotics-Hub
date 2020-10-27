using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Trajectory;

namespace RosMessageTypes.Moveit
{
    public class AttachedCollisionObject : Message
    {
        public const string RosMessageName = "moveit_msgs-master/AttachedCollisionObject";

        //  The CollisionObject will be attached with a fixed joint to this link
        public string link_name { get; set; }
        // This contains the actual shapes and poses for the CollisionObject
        // to be attached to the link
        // If action is remove and no object.id is set, all objects
        // attached to the link indicated by link_name will be removed
        public CollisionObject objectery { get; set; }
        //  The set of links that the attached objects are allowed to touch
        //  by default - the link_name is already considered by default
        public string[] touch_links { get; set; }
        //  If certain links were placed in a particular posture for this object to remain attached 
        //  (e.g., an end effector closing around an object), the posture necessary for releasing
        //  the object is stored here
        public JointTrajectory detach_posture { get; set; }
        //  The weight of the attached object, if known
        public double weight { get; set; }

        public AttachedCollisionObject()
        {
            this.link_name = "";
            this.objectery = new CollisionObject();
            this.touch_links = new string[0];
            this.detach_posture = new JointTrajectory();
            this.weight = 0.0;
        }

        public AttachedCollisionObject(string link_name, CollisionObject objectery, string[] touch_links, JointTrajectory detach_posture, double weight)
        {
            this.link_name = link_name;
            this.objectery = objectery;
            this.touch_links = touch_links;
            this.detach_posture = detach_posture;
            this.weight = weight;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.link_name));
            listOfSerializations.AddRange(objectery.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(touch_links.Length));
            foreach(var entry in touch_links)
                listOfSerializations.Add(SerializeString(entry));
            listOfSerializations.AddRange(detach_posture.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.weight));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var link_nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.link_name = DeserializeString(data, offset, link_nameStringBytesLength);
            offset += link_nameStringBytesLength;
            offset = this.objectery.Deserialize(data, offset);
            
            var touch_linksArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.touch_links= new string[touch_linksArrayLength];
            for(var i =0; i <touch_linksArrayLength; i++)
            {
                var touch_linksStringBytesLength = DeserializeLength(data, offset);
                offset += 4;
                this.touch_links[i] = DeserializeString(data, offset, touch_linksStringBytesLength);
                offset += touch_linksStringBytesLength;
            }
            offset = this.detach_posture.Deserialize(data, offset);
            this.weight = BitConverter.ToDouble(data, offset);
            offset += 8;

            return offset;
        }

    }
}
