  

using System.Collections.Generic;
using System.Linq;
using System.Xml;
using System.Xml.Linq;

namespace RosSharp.Urdf
{
    public class Link
    {
        public string name;
        public Inertial inertial;
        public List<Visual> visuals;
        public List<Collision> collisions;
        public List<Joint> joints;

        public Link(XElement node)
        {
            name = (string)node.Attribute("name");  // required
            inertial = (node.Element("inertial") != null) ? new Inertial(node.Element("inertial")) : null;  // optional     
            visuals = readVisuals(node); // multiple
            collisions = readCollisions(node); // optional   
        }

        public Link(string name, Inertial inertial = null)
        {
            this.name = name;
            this.inertial = inertial;

            visuals = new List<Visual>();
            collisions = new List<Collision>();
            joints = new List<Joint>();
        }

        public void WriteToUrdf(XmlWriter writer)
        {
            writer.WriteStartElement("link");
            writer.WriteAttributeString("name", name);

            inertial?.WriteToUrdf(writer);

            foreach (var visual in visuals)
                visual.WriteToUrdf(writer);
            foreach (var collision in collisions)
                collision.WriteToUrdf(writer);

            writer.WriteEndElement();
        }

        private static List<Collision> readCollisions(XElement node)
        {
            var collisions =
                from child in node.Elements("collision")
                select new Collision(child);
            return collisions.ToList();

        }
        private static List<Visual> readVisuals(XElement node)
        {
            var visuals =
                from child in node.Elements("visual")
                select new Visual(child);
            return visuals.ToList();
        }

        public class Inertial
        {
            public double mass;
            public Origin origin;
            public Inertia inertia;

            public Inertial(XElement node)
            {
                origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
                mass = (double)node.Element("mass").Attribute("value");// required
                inertia = new Inertia(node.Element("inertia")); // required
            }

            public Inertial(double mass, Origin origin, Inertia inertia)
            {
                this.mass = mass;
                this.origin = origin;
                this.inertia = inertia;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                writer.WriteStartElement("inertial");

                origin.WriteToUrdf(writer);

                writer.WriteStartElement("mass");
                writer.WriteAttributeString("value", mass + "");
                writer.WriteEndElement();

                inertia.WriteToUrdf(writer);

                writer.WriteEndElement();
            }

            public class Inertia
            {
                public double ixx;
                public double ixy;
                public double ixz;
                public double iyy;
                public double iyz;
                public double izz;

                public Inertia(XElement node)
                {
                    ixx = (double)node.Attribute("ixx");
                    ixy = (double)node.Attribute("ixy");
                    ixz = (double)node.Attribute("ixz");
                    iyy = (double)node.Attribute("iyy");
                    iyz = (double)node.Attribute("iyz");
                    izz = (double)node.Attribute("izz");
                }

                public Inertia(double ixx, double ixy, double ixz, double iyy, double iyz, double izz)
                {
                    this.ixx = ixx;
                    this.ixy = ixy;
                    this.ixz = ixz;
                    this.iyy = iyy;
                    this.iyz = iyz;
                    this.izz = izz;
                }

                public void WriteToUrdf(XmlWriter writer)
                {
                    writer.WriteStartElement("inertia");
                    writer.WriteAttributeString("ixx", ixx + "");
                    writer.WriteAttributeString("ixy", ixy + "");
                    writer.WriteAttributeString("ixz", ixz + "");
                    writer.WriteAttributeString("iyy", iyy + "");
                    writer.WriteAttributeString("iyz", iyz + "");
                    writer.WriteAttributeString("izz", izz + "");
                    writer.WriteEndElement();
                }

            }
        }

        public class Collision
        {
            public string name;
            public Origin origin;
            public Geometry geometry;

            public Collision(XElement node)
            {
                name = (string)node.Attribute("name"); // optional
                origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
                geometry = new Geometry(node.Element("geometry")); // required
            }

            public Collision(Geometry geometry, string name = null, Origin origin = null)
            {
                this.name = name;
                this.origin = origin;
                this.geometry = geometry;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                writer.WriteStartElement("collision");

                if (name != null)
                    writer.WriteAttributeString("name", name);

                origin?.WriteToUrdf(writer);
                geometry?.WriteToUrdf(writer);

                writer.WriteEndElement();

            }
        }
        public class Visual
        {
            public string name;
            public Origin origin;
            public Geometry geometry;
            public Material material;

            public Visual(XElement node)
            {
                name = (string)node.Attribute("name"); // optional
                origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional
                geometry = new Geometry(node.Element("geometry")); // required
                material = (node.Element("material") != null) ? new Material(node.Element("material")) : null; // optional
            }

            public Visual(Geometry geometry, string name = null, Origin origin = null, Material material = null)
            {
                this.name = name;
                this.origin = origin;
                this.geometry = geometry;
                this.material = material;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                writer.WriteStartElement("visual");

                if (name != null)
                    writer.WriteAttributeString("name", name);

                origin?.WriteToUrdf(writer);
                geometry?.WriteToUrdf(writer);
                material?.WriteToUrdf(writer);

                writer.WriteEndElement();
            }

            public class Material
            {
                public string name;
                public Color color;
                public Texture texture;

                public Material(XElement node)
                {
                    name = (string)node.Attribute("name"); // required
                    color = (node.Element("color") != null) ? new Color(node.Element("color")) : null; // optional  
                    texture = (node.Element("texture") != null) ? new Texture(node.Element("texture")) : null;
                }

                public Material(string name, Color color = null, Texture texture = null)
                {
                    this.name = name;
                    this.color = color;
                    this.texture = texture;
                }

                public void WriteToUrdf(XmlWriter writer)
                {
                    writer.WriteStartElement("material");
                    writer.WriteAttributeString("name", name);

                    color?.WriteToUrdf(writer);
                    texture?.WriteToUrdf(writer);

                    writer.WriteEndElement();
                }

                public class Texture
                {
                    public string filename;

                    public Texture(XElement node)
                    {
                        filename = (string)node.Attribute("filename"); // required
                    }

                    public Texture(string filename)
                    {
                        this.filename = filename;
                    }

                    public void WriteToUrdf(XmlWriter writer)
                    {
                        writer.WriteStartElement("texture");
                        writer.WriteAttributeString("filename", filename);
                        writer.WriteEndElement();
                    }
                }

                public class Color
                {
                    public double[] rgba;

                    public Color(XElement node)
                    {
                        rgba = node.Attribute("rgba").ReadDoubleArray(); // required
                    }

                    public Color(double[] rgba)
                    {
                        this.rgba = rgba;
                    }

                    public void WriteToUrdf(XmlWriter writer)
                    {
                        writer.WriteStartElement("color");
                        writer.WriteAttributeString("rgba", rgba.DoubleArrayToString());
                        writer.WriteEndElement();
                    }
                }

            }
        }

        public class Geometry
        {
            public Box box;
            public Cylinder cylinder;
            public Sphere sphere;
            public Mesh mesh;

            public Geometry(XElement node)
            {
                box = (node.Element("box") != null) ? new Box(node.Element("box")) : null; // optional  
                cylinder = (node.Element("cylinder") != null) ? new Cylinder(node.Element("cylinder")) : null; // optional  
                sphere = (node.Element("sphere") != null) ? new Sphere(node.Element("sphere")) : null; // optional  
                mesh = (node.Element("mesh") != null) ? new Mesh(node.Element("mesh")) : null; // optional           
            }

            public Geometry(Box box = null, Cylinder cylinder = null, Sphere sphere = null, Mesh mesh = null)
            {
                this.box = box;
                this.cylinder = cylinder;
                this.sphere = sphere;
                this.mesh = mesh;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                writer.WriteStartElement("geometry");

                box?.WriteToUrdf(writer);
                cylinder?.WriteToUrdf(writer);
                sphere?.WriteToUrdf(writer);
                mesh?.WriteToUrdf(writer);

                writer.WriteEndElement();
            }

            public class Box
            {
                public double[] size;

                public Box(XElement node)
                {
                    size = node.Attribute("size") != null ? node.Attribute("size").ReadDoubleArray() : null;
                }

                public Box(double[] size)
                {
                    this.size = size;
                }

                public void WriteToUrdf(XmlWriter writer)
                {
                    writer.WriteStartElement("box");
                    writer.WriteAttributeString("size", size.DoubleArrayToString());
                    writer.WriteEndElement();
                }
            }


            public class Cylinder
            {
                public double radius;
                public double length;

                public Cylinder(XElement node)
                {
                    radius = (double)node.Attribute("radius");
                    length = (double)node.Attribute("length");
                }

                public Cylinder(double radius, double length)
                {
                    this.radius = radius;
                    this.length = length;
                }

                public void WriteToUrdf(XmlWriter writer)
                {
                    writer.WriteStartElement("cylinder");
                    writer.WriteAttributeString("length", length + "");
                    writer.WriteAttributeString("radius", radius + "");
                    writer.WriteEndElement();
                }
            }


            public class Sphere
            {
                public double radius;

                public Sphere(XElement node)
                {
                    radius = (double)node.Attribute("radius");
                }

                public Sphere(double radius)
                {
                    this.radius = radius;
                }

                public void WriteToUrdf(XmlWriter writer)
                {
                    writer.WriteStartElement("sphere");
                    writer.WriteAttributeString("radius", radius + "");
                    writer.WriteEndElement();
                }
            }

            public class Mesh
            {
                public string filename;
                public double[] scale;

                public Mesh(XElement node)
                {
                    filename = (string)node.Attribute("filename");
                    scale = node.Attribute("scale") != null ? node.Attribute("scale").ReadDoubleArray() : null;
                }

                public Mesh(string filename, double[] scale)
                {
                    this.filename = filename;
                    this.scale = scale;
                }

                public void WriteToUrdf(XmlWriter writer)
                {
                    writer.WriteStartElement("mesh");
                    writer.WriteAttributeString("filename", filename);

                    if (scale != null)
                        writer.WriteAttributeString("scale", scale.DoubleArrayToString());

                    writer.WriteEndElement();

                }
            }
        }
    }
}
