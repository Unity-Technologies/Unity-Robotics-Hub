using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Shape;
using RosMessageTypes.Sensor;

namespace RosMessageTypes.ObjectRecognition
{
    public class ObjectInformation : Message
    {
        public const string RosMessageName = "object_recognition_msgs-master/ObjectInformation";

        // ############################################# VISUALIZATION INFO ######################################################
        // ################## THIS INFO SHOULD BE OBTAINED INDEPENDENTLY FROM THE CORE, LIKE IN AN RVIZ PLUGIN ###################
        //  The human readable name of the object
        public string name { get; set; }
        //  The full mesh of the object: this can be useful for display purposes, augmented reality ... but it can be big
        //  Make sure the type is MESH
        public Mesh ground_truth_mesh { get; set; }
        //  Sometimes, you only have a cloud in the DB
        //  Make sure the type is POINTS
        public PointCloud2 ground_truth_point_cloud { get; set; }

        public ObjectInformation()
        {
            this.name = "";
            this.ground_truth_mesh = new Mesh();
            this.ground_truth_point_cloud = new PointCloud2();
        }

        public ObjectInformation(string name, Mesh ground_truth_mesh, PointCloud2 ground_truth_point_cloud)
        {
            this.name = name;
            this.ground_truth_mesh = ground_truth_mesh;
            this.ground_truth_point_cloud = ground_truth_point_cloud;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.Add(SerializeString(this.name));
            listOfSerializations.AddRange(ground_truth_mesh.SerializationStatements());
            listOfSerializations.AddRange(ground_truth_point_cloud.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            var nameStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.name = DeserializeString(data, offset, nameStringBytesLength);
            offset += nameStringBytesLength;
            offset = this.ground_truth_mesh.Deserialize(data, offset);
            offset = this.ground_truth_point_cloud.Deserialize(data, offset);

            return offset;
        }

    }
}
