  

using System.Xml;
using System.Xml.Linq;

namespace RosSharp.Urdf
{
    public class Joint
    {
        public string name;
        public string type;
        public Origin origin;
        public string parent;
        public string child;
        public Axis axis;
        public Calibration calibration;
        public Dynamics dynamics;
        public Limit limit;
        public Mimic mimic;
        public SafetyController safetyController;

        public Link ChildLink;

        public Joint(XElement node)
        {
            name = (string)node.Attribute("name"); // required
            type = (string)node.Attribute("type"); // required
            origin = (node.Element("origin") != null) ? new Origin(node.Element("origin")) : null; // optional  
            parent = (string)node.Element("parent").Attribute("link"); // required
            child = (string)node.Element("child").Attribute("link"); // required
            axis = (node.Element("axis") != null) ? new Axis(node.Element("axis")) : null;  // optional 
            calibration = (node.Element("calibration") != null) ? new Calibration(node.Element("calibration")) : null;  // optional 
            dynamics = (node.Element("dynamics") != null) ? new Dynamics(node.Element("dynamics")) : null;  // optional 
            limit = (node.Element("limit") != null) ? new Limit(node.Element("limit")) : null;  // required only for revolute and prismatic joints
            mimic = (node.Element("mimic") != null) ? new Mimic(node.Element("mimic")) : null;  // optional
            safetyController = (node.Element("safety_controller") != null) ? new SafetyController(node.Element("safety_controller")) : null;  // optional
        }

        public Joint(string name, string type, string parent, string child,
            Origin origin = null, Axis axis = null, Calibration calibration = null,
            Dynamics dynamics = null, Limit limit = null, Mimic mimic = null, SafetyController safetyController = null)
        {
            this.name = name;
            this.type = type;
            this.parent = parent;
            this.child = child;
            this.origin = origin;
            this.axis = axis;
            this.calibration = calibration;
            this.dynamics = dynamics;
            this.limit = limit;
            this.mimic = mimic;
            this.safetyController = safetyController;
        }

        public void WriteToUrdf(XmlWriter writer)
        {
            writer.WriteStartElement("joint");

            writer.WriteAttributeString("name", name);
            writer.WriteAttributeString("type", type);

            origin?.WriteToUrdf(writer);

            writer.WriteStartElement("parent");
            writer.WriteAttributeString("link", parent);
            writer.WriteEndElement();

            writer.WriteStartElement("child");
            writer.WriteAttributeString("link", child);
            writer.WriteEndElement();

            axis?.WriteToUrdf(writer);
            calibration?.WriteToUrdf(writer);
            dynamics?.WriteToUrdf(writer);
            limit?.WriteToUrdf(writer);
            mimic?.WriteToUrdf(writer);
            safetyController?.WriteToUrdf(writer);

            writer.WriteEndElement();
        }

        public class Axis
        {
            public double[] xyz;

            public Axis(XElement node)
            {
                xyz = node.Attribute("xyz") != null ? node.Attribute("xyz").ReadDoubleArray() : null;
            }

            public Axis(double[] xyz)
            {
                this.xyz = xyz;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                if (!(xyz[0] == 0 && xyz[1] == 0 && xyz[2] == 0))
                {
                    writer.WriteStartElement("axis");
                    writer.WriteAttributeString("xyz", xyz.DoubleArrayToString());
                    writer.WriteEndElement();
                }
            }

            public int AxisofMotion(){
                for(int i = 0; i < 3; i++){
                    if(xyz[i] > 0){
                        return i;
                    }
                }
                return -1;
            }
        }

        public class Calibration
        {
            public double rising;
            public double falling;

            public Calibration(XElement node)
            {
                rising = node.Attribute("rising").ReadOptionalDouble();  // optional
                falling = node.Attribute("falling").ReadOptionalDouble();  // optional
            }

            public Calibration(double rising = 0, double falling = 0)
            {
                this.rising = rising;
                this.falling = falling;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                writer.WriteStartElement("calibration");

                //TODO only output one or the other
                writer.WriteAttributeString("rising", rising + "");
                writer.WriteAttributeString("falling", falling + "");

                writer.WriteEndElement();
            }
        }

        public class Dynamics
        {
            public double damping;
            public double friction;

            public Dynamics(XElement node)
            {
                damping = node.Attribute("damping").ReadOptionalDouble(); // optional
                friction = node.Attribute("friction").ReadOptionalDouble(); // optional
            }

            public Dynamics(double damping, double friction)
            {
                this.damping = damping;
                this.friction = friction;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                if (damping == 0 & friction == 0)
                    return;

                writer.WriteStartElement("dynamics");

                if (damping != 0)
                    writer.WriteAttributeString("damping", damping + "");
                if (friction != 0)
                    writer.WriteAttributeString("friction", friction + "");

                writer.WriteEndElement();
            }
        }

        public class Limit
        {
            public double lower;
            public double upper;
            public double effort;
            public double velocity;

            public Limit(XElement node)
            {
                lower = node.Attribute("lower").ReadOptionalDouble(); // optional
                upper = node.Attribute("upper").ReadOptionalDouble(); // optional
                effort = (double)node.Attribute("effort"); // required
                velocity = (double)node.Attribute("velocity"); // required
            }
            
            public Limit(double lower, double upper, double effort, double velocity)
            {
                this.lower = lower;
                this.upper = upper;
                this.effort = effort;
                this.velocity = velocity;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                writer.WriteStartElement("limit");

                writer.WriteAttributeString("lower", lower + "");
                writer.WriteAttributeString("upper", upper + "");

                writer.WriteAttributeString("effort", effort + "");
                writer.WriteAttributeString("velocity", velocity + "");

                writer.WriteEndElement();
            }
        }

        public class Mimic
        {
            public string joint;
            public double multiplier;
            public double offset;

            public Mimic(XElement node)
            {
                joint = (string)node.Attribute("joint"); // required
                multiplier = node.Attribute("multiplier").ReadOptionalDouble(); // optional
                offset = node.Attribute("offset").ReadOptionalDouble(); // optional   
            }

            public Mimic(string joint, double multiplier = 0, double offset = 0)
            {
                this.joint = joint;
                this.multiplier = multiplier;
                this.offset = offset;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                if (multiplier == 1 && offset == 0)
                    return;

                writer.WriteStartElement("mimic");

                writer.WriteAttributeString("joint", joint);
                if (multiplier != 1)
                    writer.WriteAttributeString("multiplier", multiplier + "");
                if (offset != 0)
                    writer.WriteAttributeString("offset", offset + "");

                writer.WriteEndElement();
            }
        }

        public class SafetyController
        {
            public double softLowerLimit;
            public double softUpperLimit;
            public double kPosition;
            public double kVelocity;

            public SafetyController(XElement node)
            {
                softLowerLimit = node.Attribute("soft_lower_limit").ReadOptionalDouble(); // optional
                softUpperLimit = node.Attribute("soft_upper_limit").ReadOptionalDouble(); // optional
                kPosition = node.Attribute("k_position").ReadOptionalDouble(); // optional
                kVelocity = node.Attribute("k_velocity").ReadOptionalDouble(); // required   
            }

            public SafetyController(double softLowerLimit, double softUpperLimit, double kPosition, double kVelocity)
            {
                this.softLowerLimit = softLowerLimit;
                this.softUpperLimit = softUpperLimit;
                this.kPosition = kPosition;
                this.kVelocity = kVelocity;
            }

            public void WriteToUrdf(XmlWriter writer)
            {
                writer.WriteStartElement("safetyController");

                if (softLowerLimit != 0)
                    writer.WriteAttributeString("soft_lower_limit", softLowerLimit + "");
                if (softUpperLimit != 0)
                    writer.WriteAttributeString("soft_upper_limit", softUpperLimit + "");
                if (kPosition != 0)
                    writer.WriteAttributeString("k_position", kPosition + "");
                writer.WriteAttributeString("k_velocity", kVelocity + "");

                writer.WriteEndElement();
            }
        }
    }

}
