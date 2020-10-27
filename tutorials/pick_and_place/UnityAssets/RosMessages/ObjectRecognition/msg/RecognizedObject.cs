using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.ObjectRecognition;
using RosMessageTypes.Sensor;
using RosMessageTypes.Shape;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.ObjectRecognition
{
    public class RecognizedObject : Message
    {
        public const string RosMessageName = "object_recognition_msgs-master/RecognizedObject";

        // #################################################### HEADER ###########################################################
        //  The header frame corresponds to the pose frame, NOT the point_cloud frame.
        public Header header { get; set; }
        // ################################################# OBJECT INFO #########################################################
        //  Contains information about the type and the position of a found object
        //  Some of those fields might not be filled because the used techniques do not fill them or because the user does not
        //  request them
        //  The type of the found object
        public ObjectType type { get; set; }
        // confidence: how sure you are it is that object and not another one.
        //  It is between 0 and 1 and the closer to one it is the better
        public float confidence { get; set; }
        // ############################################### OBJECT CLUSTERS #######################################################
        //  Sometimes you can extract the 3d points that belong to the object, in the frames of the original sensors
        //  (it is an array as you might have several sensors)
        public PointCloud2[] point_clouds { get; set; }
        //  Sometimes, you can only provide a bounding box/shape, even in 3d
        //  This is in the pose frame
        public Mesh bounding_mesh { get; set; }
        //  Sometimes, you only have 2d input so you can't really give a pose, you just get a contour, or a box
        //  The last point will be linked to the first one automatically
        public Point[] bounding_contours { get; set; }
        // ################################################### POSE INFO #########################################################
        //  This is the result that everybody expects : the pose in some frame given with the input. The units are radian/meters
        //  as usual
        public PoseWithCovarianceStamped pose { get; set; }

        public RecognizedObject()
        {
            this.header = new Header();
            this.type = new ObjectType();
            this.confidence = 0.0f;
            this.point_clouds = new PointCloud2[0];
            this.bounding_mesh = new Mesh();
            this.bounding_contours = new Point[0];
            this.pose = new PoseWithCovarianceStamped();
        }

        public RecognizedObject(Header header, ObjectType type, float confidence, PointCloud2[] point_clouds, Mesh bounding_mesh, Point[] bounding_contours, PoseWithCovarianceStamped pose)
        {
            this.header = header;
            this.type = type;
            this.confidence = confidence;
            this.point_clouds = point_clouds;
            this.bounding_mesh = bounding_mesh;
            this.bounding_contours = bounding_contours;
            this.pose = pose;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.AddRange(type.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.confidence));
            
            listOfSerializations.Add(BitConverter.GetBytes(point_clouds.Length));
            foreach(var entry in point_clouds)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(bounding_mesh.SerializationStatements());
            
            listOfSerializations.Add(BitConverter.GetBytes(bounding_contours.Length));
            foreach(var entry in bounding_contours)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.AddRange(pose.SerializationStatements());

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            offset = this.type.Deserialize(data, offset);
            this.confidence = BitConverter.ToSingle(data, offset);
            offset += 4;
            
            var point_cloudsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.point_clouds= new PointCloud2[point_cloudsArrayLength];
            for(var i =0; i <point_cloudsArrayLength; i++)
            {
                this.point_clouds[i] = new PointCloud2();
                offset = this.point_clouds[i].Deserialize(data, offset);
            }
            offset = this.bounding_mesh.Deserialize(data, offset);
            
            var bounding_contoursArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.bounding_contours= new Point[bounding_contoursArrayLength];
            for(var i =0; i <bounding_contoursArrayLength; i++)
            {
                this.bounding_contours[i] = new Point();
                offset = this.bounding_contours[i].Deserialize(data, offset);
            }
            offset = this.pose.Deserialize(data, offset);

            return offset;
        }

    }
}
