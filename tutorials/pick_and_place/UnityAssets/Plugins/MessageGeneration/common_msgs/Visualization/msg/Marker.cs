using System;
using System.Collections.Generic;
using System.Text;
using RosMessageGeneration;
using RosMessageTypes.Std;
using RosMessageTypes.Geometry;

namespace RosMessageTypes.Visualization
{
    public class Marker : Message
    {
        public const string RosMessageName = "visualization_msgs/Marker";

        //  See http://www.ros.org/wiki/rviz/DisplayTypes/Marker and http://www.ros.org/wiki/rviz/Tutorials/Markers%3A%20Basic%20Shapes for more information on using this message with rviz
        public const byte ARROW = 0;
        public const byte CUBE = 1;
        public const byte SPHERE = 2;
        public const byte CYLINDER = 3;
        public const byte LINE_STRIP = 4;
        public const byte LINE_LIST = 5;
        public const byte CUBE_LIST = 6;
        public const byte SPHERE_LIST = 7;
        public const byte POINTS = 8;
        public const byte TEXT_VIEW_FACING = 9;
        public const byte MESH_RESOURCE = 10;
        public const byte TRIANGLE_LIST = 11;
        public const byte ADD = 0;
        public const byte MODIFY = 0;
        public const byte DELETE = 2;
        public const byte DELETEALL = 3;
        public Header header { get; set; }
        //  header for time/frame information
        public string ns { get; set; }
        //  Namespace to place this object in... used in conjunction with id to create a unique name for the object
        public int id { get; set; }
        //  object ID useful in conjunction with the namespace for manipulating and deleting the object later
        public int type { get; set; }
        //  Type of object
        public int action { get; set; }
        //  0 add/modify an object, 1 (deprecated), 2 deletes an object, 3 deletes all objects
        public Pose pose { get; set; }
        //  Pose of the object
        public Vector3 scale { get; set; }
        //  Scale of the object 1,1,1 means default (usually 1 meter square)
        public ColorRGBA color { get; set; }
        //  Color [0.0-1.0]
        public Duration lifetime { get; set; }
        //  How long the object should last before being automatically deleted.  0 means forever
        public bool frame_locked { get; set; }
        //  If this marker should be frame-locked, i.e. retransformed into its frame every timestep
        // Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
        public Point[] points { get; set; }
        // Only used if the type specified has some use for them (eg. POINTS, LINE_STRIP, ...)
        // number of colors must either be 0 or equal to the number of points
        // NOTE: alpha is not yet used
        public ColorRGBA[] colors { get; set; }
        //  NOTE: only used for text markers
        public string text { get; set; }
        //  NOTE: only used for MESH_RESOURCE markers
        public string mesh_resource { get; set; }
        public bool mesh_use_embedded_materials { get; set; }

        public Marker()
        {
            this.header = new Header();
            this.ns = "";
            this.id = 0;
            this.type = 0;
            this.action = 0;
            this.pose = new Pose();
            this.scale = new Vector3();
            this.color = new ColorRGBA();
            this.lifetime = new Duration();
            this.frame_locked = false;
            this.points = new Point[0];
            this.colors = new ColorRGBA[0];
            this.text = "";
            this.mesh_resource = "";
            this.mesh_use_embedded_materials = false;
        }

        public Marker(Header header, string ns, int id, int type, int action, Pose pose, Vector3 scale, ColorRGBA color, Duration lifetime, bool frame_locked, Point[] points, ColorRGBA[] colors, string text, string mesh_resource, bool mesh_use_embedded_materials)
        {
            this.header = header;
            this.ns = ns;
            this.id = id;
            this.type = type;
            this.action = action;
            this.pose = pose;
            this.scale = scale;
            this.color = color;
            this.lifetime = lifetime;
            this.frame_locked = frame_locked;
            this.points = points;
            this.colors = colors;
            this.text = text;
            this.mesh_resource = mesh_resource;
            this.mesh_use_embedded_materials = mesh_use_embedded_materials;
        }
        public override List<byte[]> SerializationStatements()
        {
            var listOfSerializations = new List<byte[]>();
            listOfSerializations.AddRange(header.SerializationStatements());
            listOfSerializations.Add(SerializeString(this.ns));
            listOfSerializations.Add(BitConverter.GetBytes(this.id));
            listOfSerializations.Add(BitConverter.GetBytes(this.type));
            listOfSerializations.Add(BitConverter.GetBytes(this.action));
            listOfSerializations.AddRange(pose.SerializationStatements());
            listOfSerializations.AddRange(scale.SerializationStatements());
            listOfSerializations.AddRange(color.SerializationStatements());
            listOfSerializations.AddRange(lifetime.SerializationStatements());
            listOfSerializations.Add(BitConverter.GetBytes(this.frame_locked));
            
            listOfSerializations.Add(BitConverter.GetBytes(points.Length));
            foreach(var entry in points)
                listOfSerializations.Add(entry.Serialize());
            
            listOfSerializations.Add(BitConverter.GetBytes(colors.Length));
            foreach(var entry in colors)
                listOfSerializations.Add(entry.Serialize());
            listOfSerializations.Add(SerializeString(this.text));
            listOfSerializations.Add(SerializeString(this.mesh_resource));
            listOfSerializations.Add(BitConverter.GetBytes(this.mesh_use_embedded_materials));

            return listOfSerializations;
        }

        public override int Deserialize(byte[] data, int offset)
        {
            offset = this.header.Deserialize(data, offset);
            var nsStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.ns = DeserializeString(data, offset, nsStringBytesLength);
            offset += nsStringBytesLength;
            this.id = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.type = BitConverter.ToInt32(data, offset);
            offset += 4;
            this.action = BitConverter.ToInt32(data, offset);
            offset += 4;
            offset = this.pose.Deserialize(data, offset);
            offset = this.scale.Deserialize(data, offset);
            offset = this.color.Deserialize(data, offset);
            offset = this.lifetime.Deserialize(data, offset);
            this.frame_locked = BitConverter.ToBoolean(data, offset);
            offset += 1;
            
            var pointsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.points= new Point[pointsArrayLength];
            for(var i =0; i <pointsArrayLength; i++)
            {
                this.points[i] = new Point();
                offset = this.points[i].Deserialize(data, offset);
            }
            
            var colorsArrayLength = DeserializeLength(data, offset);
            offset += 4;
            this.colors= new ColorRGBA[colorsArrayLength];
            for(var i =0; i <colorsArrayLength; i++)
            {
                this.colors[i] = new ColorRGBA();
                offset = this.colors[i].Deserialize(data, offset);
            }
            var textStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.text = DeserializeString(data, offset, textStringBytesLength);
            offset += textStringBytesLength;
            var mesh_resourceStringBytesLength = DeserializeLength(data, offset);
            offset += 4;
            this.mesh_resource = DeserializeString(data, offset, mesh_resourceStringBytesLength);
            offset += mesh_resourceStringBytesLength;
            this.mesh_use_embedded_materials = BitConverter.ToBoolean(data, offset);
            offset += 1;

            return offset;
        }

    }
}
