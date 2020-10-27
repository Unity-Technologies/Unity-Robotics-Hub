  

using System.Xml;
using System.Xml.Linq;

namespace RosSharp.Urdf
{
    public class Plugin
    {
        public string text;

        public Plugin(XElement node)
        {
            text = node.ToString();
        }

        public Plugin(string text)
        {
            this.text = text;
        }

        public void WriteToUrdf(XmlWriter writer)
        {
            XmlDocument xDoc = new XmlDocument {PreserveWhitespace = true};
            xDoc.LoadXml(text);
            xDoc.WriteContentTo(writer);
            writer.WriteWhitespace("\n");
        }
    }
}
