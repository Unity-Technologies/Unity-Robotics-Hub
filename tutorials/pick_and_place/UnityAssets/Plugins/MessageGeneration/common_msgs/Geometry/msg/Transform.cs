using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;

namespace RosMessageTypes.Geometry
{
    public class Transform : Message
    {
        public const string RosMessageName = "geometry_msgs/Transform";

        //  This represents the transform between two coordinate frames in free space.
        public Vector3 translation { get; set; }
        public Quaternion rotation { get; set; }

        public Transform()
        {
            this.translation = new Vector3();
            this.rotation = new Quaternion();
        }

        public Transform(Vector3 translation, Quaternion rotation)
        {
            this.translation = translation;
            this.rotation = rotation;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(translation.SerializationStatements());
            listOfSerializations.AddRange(rotation.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.translation.Deserialize(data, offset);
            offset = this.rotation.Deserialize(data, offset);

            return offset;
        }

    }
}
