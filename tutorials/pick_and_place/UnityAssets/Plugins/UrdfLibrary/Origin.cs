  

using System.Xml;
using System.Xml.Linq;

namespace RosSharp.Urdf
{
    public class Origin
    {
        public double[] Xyz;
        public double[] Rpy;

        public Origin(XElement node)
        {
            Xyz = node.Attribute("xyz") != null ? node.Attribute("xyz").ReadDoubleArray() : null;
            Rpy = node.Attribute("rpy") != null ? node.Attribute("rpy").ReadDoubleArray() : null;
        }

        public Origin(double[] xyz, double[] rpy)
        {
            Xyz = xyz;
            Rpy = rpy;
        }

        public void WriteToUrdf(XmlWriter writer)
        {
            writer.WriteStartElement("origin");
            if(Rpy != null)
                writer.WriteAttributeString("rpy", Rpy.DoubleArrayToString());
            if(Xyz != null)
                writer.WriteAttributeString("xyz", Xyz.DoubleArrayToString());
            writer.WriteEndElement();
        }
    }
}
