   

using System.Collections.Generic;
using System.IO;
using System.Linq;
using System.Threading;
using System.Xml;
using System.Xml.Linq;
using System;


namespace RosSharp.Urdf
{
    public class Robot
    {
        public string filename;
        public string name;
        public Link root;
        public List<Link.Visual.Material> materials;

        public List<Link> links;
        public List<Joint> joints;
        public List<Plugin> plugins;

        public Robot(string filename)
        {
            
            this.filename = filename;
            
            XDocument xdoc = XDocument.Load(filename);
            XElement node = xdoc.Element("robot");

            name = node.Attribute("name").Value;
            materials = ReadMaterials(node);
            links = ReadLinks(node);
            joints = ReadJoints(node);
            plugins = ReadPlugins(node);
            

            // build tree structure from link and joint lists:
            foreach (Link link in links)
                link.joints = joints.FindAll(v => v.parent == link.name);
            foreach (Joint joint in joints)
                joint.ChildLink = links.Find(v => v.name == joint.child);

            // save root node only:
            root = FindRootLink(links, joints);
            
        }

        public Robot(string filename, string name)
        {
            this.filename = filename;
            this.name = name;

            links = new List<Link>();
            joints = new List<Joint>();
            plugins = new List<Plugin>();
            materials = new List<Link.Visual.Material>();
        }

        private static List<Link.Visual.Material> ReadMaterials(XElement node)
        {
            var materials =
                from child in node.Elements("material")
                select new Link.Visual.Material(child);
            return materials.ToList();
        }

        private static List<Link> ReadLinks(XElement node)
        {
            var links =
                from child in node.Elements("link")
                select new Link(child);
            return links.ToList();
        }

        private static List<Joint> ReadJoints(XElement node)
        {
            var joints =
                from child in node.Elements("joint")
                select new Joint(child);
            return joints.ToList();
        }

        private List<Plugin> ReadPlugins(XElement node)
        {
            var plugins =
                from child in node.Elements()
                where child.Name != "link" && child.Name != "joint" && child.Name != "material"
                select new Plugin(child.ToString());
            return plugins.ToList();
        }

        private static Link FindRootLink(List<Link> links, List<Joint> joints)
        {
            if (joints.Count == 0)
                return links[0];

            Joint joint = joints[0];
            string parent;
            do
            {
                parent = joint.parent;
                joint = joints.Find(v => v.child == parent);
            }
            while (joint != null);
            return links.Find(v => v.name == parent);
        }

        public void WriteToUrdf()
        {
            // executing writeToUrdf() in separate thread to ensure CultureInfo does not change in main thread
            Thread thread = new Thread(delegate () { writeToUrdf(); });
            thread.Start();
            thread.Join();
        }
        private void writeToUrdf()
        {
            Thread.CurrentThread.CurrentCulture = System.Globalization.CultureInfo.InvariantCulture;
            Directory.CreateDirectory(Path.GetDirectoryName(filename));
            XmlWriterSettings settings = new XmlWriterSettings { Indent = true, NewLineOnAttributes = false };

            using (XmlWriter writer = XmlWriter.Create(filename, settings))
            {
                writer.WriteStartDocument();
                writer.WriteStartElement("robot");
                writer.WriteAttributeString("name", name);

                foreach (var material in materials)
                    material.WriteToUrdf(writer);
                foreach (var link in links)
                    link.WriteToUrdf(writer);
                foreach (var joint in joints)
                    joint.WriteToUrdf(writer);
                foreach (var plugin in plugins)
                    plugin.WriteToUrdf(writer);
                
                writer.WriteEndElement();
                writer.WriteEndDocument();

                writer.Close();
            }
        }
    }
}
