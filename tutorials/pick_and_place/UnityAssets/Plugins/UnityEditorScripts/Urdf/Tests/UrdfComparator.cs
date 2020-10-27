using System;
using System.Collections;
using System.Collections.Generic;
using System.IO;
using UnityEngine;

namespace RosSharp.Urdf.Testing
{
    public class UrdfComparator
    {
        Robot source;
        Robot exported;
        string logFileName = "";
        System.Text.StringBuilder jointLog;
        System.Text.StringBuilder linkLog;

        public UrdfComparator(string sourceFilePath, string exportedFilePath, string logFilePath)
        {
            this.source = new Robot(sourceFilePath);
            this.exported = new Robot(exportedFilePath);
            this.logFileName = @logFilePath + "/" + Path.GetFileName(sourceFilePath) + "_" + Path.GetFileName(exportedFilePath)
                                   + "_" + DateTime.Now.TimeOfDay + ".txt";
            Debug.Log("Compared Log Filepath :" + logFilePath);

            if (!File.Exists(logFileName))
            {
                using (StreamWriter sw = File.AppendText(logFileName))
                {
                    sw.WriteLine("************** String Comparator ************\n");
                }
            }

            jointLog = new System.Text.StringBuilder();
            linkLog = new System.Text.StringBuilder();
        }

        public bool Compare()
        {
            using (StreamWriter sw = File.AppendText(logFileName))
            {
                sw.WriteLine(String.Format("No of Links :"));
                sw.WriteLine(String.Format("Equal:{0,6}", source.links.Count == exported.links.Count));
                sw.WriteLine(String.Format("Count: Source:{0,5:D4} Exported:{1,5:D4}", source.links.Count, exported.links.Count));
            }
            if (source.links.Count != exported.links.Count)
                return false;


            using (StreamWriter sw = File.AppendText(logFileName))
            {
                sw.WriteLine(String.Format("No of Joints:"));
                sw.WriteLine(String.Format(" Equal:{0,6}", source.joints.Count == exported.joints.Count));
                sw.WriteLine(String.Format("Count:{0,5:D4}", source.joints.Count));
            }
            if (source.joints.Count != exported.joints.Count)
                return false;

            bool flag;

            if(CompareLink(this.source.root, this.exported.root,0))
            {
                flag = true;
            }
            else
            {
                flag = false;
            }

            using(StreamWriter sw = File.AppendText(logFileName))
            {
                sw.WriteLine(linkLog);
                sw.WriteLine(jointLog);
            }


            return flag;
        }

        /// <summary>
        /// Compares two links of a robot
        /// </summary>
        /// <param name="source">First link to be compared</param>
        /// <param name="exported">Second link to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareLink( Link source,  Link exported, int indent)
        {
            linkLog.AppendLine("\n\n********LINK*****\n");

            bool linkNameEqual = (source.name == exported.name);
            linkLog.AppendLine(String.Format("{0}Name:", Indent(indent + 1)));
            linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent + 1), linkNameEqual));
            linkLog.AppendLine(String.Format("{0}Name: Source: {1,12} Exported: {2,12}", Indent(indent + 1), source.name, exported.name));
            if (!linkNameEqual)
            {
                return false;
            }

            linkLog.AppendLine(String.Format("{0}Inertial Checks",Indent(indent + 1)));

            if (!CompareInertial( source.inertial,  exported.inertial, indent +2))
                return false;

            linkLog.AppendLine(String.Format("{0}Visual Checks", Indent(indent + 1)));

            bool visualCountsEqual = (source.visuals.Count == exported.visuals.Count);
            linkLog.AppendLine(String.Format("{0}Number of Visual Components Equal:{1,6}", Indent(indent + 1), visualCountsEqual));
            if (visualCountsEqual)
            {
                for (int i = 0; i < source.visuals.Count; i++)
                {
                    linkLog.AppendLine(String.Format("{0}-Visual Component: {1,3:d}", Indent(indent + 1), i+1));
                    if (CompareVisual(source.visuals[i], exported.visuals[i],indent+2))
                    {
                        continue;
                    }
                    else
                    {
                        return false;
                    }
                }
       
            }
            else
            {
                return false;
            }

            linkLog.AppendLine(String.Format("{0}Collisions Checks", Indent(indent + 1)));

            bool collisionCountEqual = (source.collisions.Count == exported.collisions.Count);
            linkLog.AppendLine(String.Format("{0}Number of Collision Components", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent + 1), collisionCountEqual));
            linkLog.AppendLine(String.Format("{0}Source Count: {1,4:d4}", Indent(indent), source.collisions.Count));
            linkLog.AppendLine(String.Format("{0}Exported Count: {1,4:d4}", Indent(indent), exported.collisions.Count));
            if (collisionCountEqual)
            {
                for (int i = 0; i < source.collisions.Count; i++)
                {
                    linkLog.AppendLine(String.Format("{0}-Collision Component: {1,3:d}", Indent(indent + 1), i + 1));
                    if (CompareCollisions(source.collisions[i], exported.collisions[i], indent +2))
                    {
                        continue;
                    }
                    else
                    {
                        return false;
                    }
                }

            }
            else
            {
                return false;
            }

            bool jointCountEqual = (source.joints.Count == exported.joints.Count);
            linkLog.AppendLine(String.Format("{0}Number of Connected Joints:", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Components Equal:{1,6}", Indent(indent), jointCountEqual));
            linkLog.AppendLine(String.Format("{0}Count: Source: {1,4:d4}", Indent(indent), source.joints.Count));
            linkLog.AppendLine(String.Format("{0}Exported: {1,4:d4}", Indent(indent + 1), exported.joints.Count));
            if (!jointCountEqual)
            {
                return false;
            }

            foreach (Joint jointSource in source.joints)
            {
                Joint jointExported = exported.joints.Find(x => x.name == jointSource.name); // Check for no match
                if(jointExported == null)
                {
                    linkLog.AppendLine(String.Format("{0}Joint Not Found in Exported: Joint Name:{1,12}",Indent(indent + 1),jointSource.name));
                    return false;
                }
                if (jointExported != null && !CompareJoint(jointSource, jointExported, indent))
                {
                    return false;
                }
            }

            return true;
        }

        /// <summary>
        /// Compares two joint of a robot
        /// </summary>
        /// <param name="source">First joint to be compared</param>
        /// <param name="exported">Second joint to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareJoint( Joint source,  Joint exported, int indent) // This function does not test for Mimic, Calibration and SafetyController as they are not imported in Unity
        {
            linkLog.AppendLine("\n\n********Joint*****\n");

            bool jointNameEqual = (source.name == exported.name);
            linkLog.AppendLine(String.Format("{0}Name:", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent), jointNameEqual));
            linkLog.AppendLine(String.Format("{0}Name: Source: {1,12}", Indent(indent), source.name));
            linkLog.AppendLine(String.Format("{0}Exported: {1,12}", Indent(indent), exported.name));
            if (!jointNameEqual)
            {
                return false;
            }

            bool jointTypeEqual = (source.type == exported.type);
            linkLog.AppendLine(String.Format("{0}Name:", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent), jointTypeEqual));
            linkLog.AppendLine(String.Format("{0}Type: Source: {1,12}", Indent(indent), source.type));
            linkLog.AppendLine(String.Format("{0}Type: Exported: {1,12}", Indent(indent), exported.type));
            if (!jointTypeEqual)
            {
                return false;
            }

            if (!CompareOrigin(source.origin, exported.origin, indent + 2))
            {
                return false;
            }

            bool parentNameEqual = (source.parent == exported.parent);
            linkLog.AppendLine(String.Format("{0}Parent:", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent), parentNameEqual));
            linkLog.AppendLine(String.Format("{0}Parent: Source: {1,12}", Indent(indent), source.parent));
            linkLog.AppendLine(String.Format("{0}Parent: Exported: {1,12}", Indent(indent), exported.parent));
            if (!parentNameEqual)
            {
                return false;
            }
   
            if (!CompareAxis(source.axis, exported.axis,indent + 2))
            {
                return false;
            }

            if (!CompareDynamics(source.dynamics, exported.dynamics,indent + 2))
            {
                return false;
            }
            if (source.type == "revolute" || source.type == "revolute")
            {
                if (!CompareLimit(source.limit, exported.limit, indent + 2))
                {
                    return false;
                }
            }

            bool childNameEqual = (source.child == exported.child);
            linkLog.AppendLine(String.Format("{0}Child Name:", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), childNameEqual));
            linkLog.AppendLine(String.Format("{0}Child Name: Source: {1,12}", Indent(indent), source.child));
            linkLog.AppendLine(String.Format("{0}Child Name: Exported: {1,12}", Indent(indent), exported.child));
            if (!childNameEqual)
            {
                return false;
            }
            else if (source.child != null)
            {
                if (!CompareLink(source.ChildLink, exported.ChildLink, indent))
                {
                    linkLog.AppendLine(String.Format("{0}Child Attributes: {1,6} Child Name: Source: {2,12} Exported: {3,12}", Indent(indent + 1), "False", source.child, exported.child));
                    return false;
                }
                linkLog.AppendLine(String.Format("{0}Child Attributes: {1,6} Child Name: {2,12}", Indent(indent + 1), "True", source.child));
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Child Attributes: {1,6} Child Name: {2,12}", Indent(indent + 1), "True", source.child));
            }
            return true;
        }

        /// <summary>
        /// Compares inertial information of two links
        /// </summary>
        /// <param name="source">First link's inertial information to be compared</param>
        /// <param name="exported">Second link's inertial information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareInertial(Link.Inertial source, Link.Inertial exported, int indent)
        {
            if (source == null && exported == null)
            {
                linkLog.AppendLine(String.Format("{0}Inertia Attributes Null:{1,6}",Indent(indent),"True"));
                return true;
            }
            if((source == null && exported !=null) || (source != null && exported == null))
            {
                linkLog.AppendLine(String.Format("{0}Inertia Attributes Null:{1,6}", Indent(indent), "False"));
                return false;
            }

            bool massEqual = source.mass.EqualsDelta(exported.mass, .05);
            linkLog.AppendLine(String.Format("{0}Mass:", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent), massEqual));
            if (!massEqual)
            {
                
                return false;
            }

            if (!CompareOrigin( source.origin,  exported.origin, indent+1))
                return false;

            return true;

        }

        /// <summary>
        /// Compares two origin information of two links
        /// </summary>
        /// <param name="source">First link's origin information to be compared</param>
        /// <param name="exported">Second link's origin information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareOrigin( Origin source,  Origin exported, int indent)
        {
            linkLog.AppendLine(String.Format("{0}Origin Checks", Indent(indent)));

            //Origin Nullity Checks
            if ((source == null && exported == null))
            {
                linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "True"));
                return true;
            }
            else if(source != null && exported == null)
            {
                if (source.Xyz.ToVector3() == new Vector3(0, 0, 0) && source.Rpy.ToVector3() == new Vector3(0, 0, 0))
                {
                    linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "True"));
                    return true;
                }
                else
                {
                    linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "False"));
                    return false;
                }
            }
            else if (exported != null && source == null)
            {
                if (exported.Xyz.ToVector3() == new Vector3(0, 0, 0) && exported.Rpy.ToVector3() == new Vector3(0, 0, 0))
                {
                    linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "True"));
                    return true;
                }
                else
                {
                    linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "False"));
                    return false;
                }
            }
            else if(source != null && exported != null)
            {
                linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "True"));
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "False"));
                return false;
            }

            // XYZ checks
            
            if ((source.Xyz == null && exported.Xyz == null) || (source.Xyz == null && exported.Xyz.ToVector3() == new Vector3(0,0,0)) || (source.Xyz.ToVector3() == new Vector3(0, 0, 0) && exported.Xyz == null))
            {
                linkLog.AppendLine(String.Format("{0}XYZ:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal Check: {1,6}", Indent(indent + 1), "True"));
                linkLog.AppendLine(String.Format("{0}Value: (0,0,0)", Indent(indent)));
            }
            else
            {
                if (source.Xyz != null && exported.Xyz != null)
                {
                    if (!exported.Xyz.DoubleDeltaCompare(exported.Xyz,0.01))
                    {
                        linkLog.AppendLine(String.Format("{0}XYZ position", Indent(indent)));
                        linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent), "False"));
                        linkLog.AppendLine(String.Format("{0}Exported: {1,10} Source:{2,10}", Indent(indent + 1), source.Xyz.ToVector3(), exported.Xyz.ToVector3()));
                        return false;
                    }
                    else
                    {
                        linkLog.AppendLine(String.Format("{0}XYZ position", Indent(indent)));
                        linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent), "True"));
                        linkLog.AppendLine(String.Format("{0}Value: {1,10}", Indent(indent + 1), source.Xyz.ToVector3()));
                    }
                }
                else
                {
                    linkLog.AppendLine(String.Format("{0}XYZ Nullity Check: {1,6}", Indent(indent + 1), "False"));
                    return false;
                }
            }
            //RPY checks
            if ((source.Rpy == null && exported.Rpy == null) || (source.Rpy == null && exported.Rpy.ToVector3() == new Vector3(0, 0, 0)) || (source.Rpy.ToVector3() == new Vector3(0, 0, 0) && exported.Rpy == null))
            {
                linkLog.AppendLine(String.Format("{0}RPY:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal Check: {1,6}", Indent(indent + 1), "True"));
                linkLog.AppendLine(String.Format("{0}Value:{1}",Indent(indent+1) ,"Zero Vector"));
            }
            else
            {
                if (source.Rpy != null && exported.Rpy != null)
                {
                    if (!RpyCheck(source.Rpy,exported.Rpy,.05))
                    {
                        linkLog.AppendLine(String.Format("{0}RPY position:", Indent(indent + 1)));
                        linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent + 1),"False"));
                        linkLog.AppendLine(String.Format("{0}Exported: {1,10} Source:{2,10}", Indent(indent + 1),source.Rpy.ToVector3(), exported.Rpy.ToVector3()));
                        return false;
                    }
                    else
                    {
                        linkLog.AppendLine(String.Format("{0}RPY position:", Indent(indent + 1)));
                        linkLog.AppendLine(String.Format("{0}Equal:{1,6}", Indent(indent + 1), "True"));
                        linkLog.AppendLine(String.Format("{0}Value: {1,10}", Indent(indent + 1), source.Rpy.ToVector3()));
                    }
                }
                else
                {
                    linkLog.AppendLine(String.Format("{0}RPY Nullity Check: {1,6}", Indent(indent + 1), "False"));
                    return false;
                }
            }
            return true;
        }

        /// <summary>
        /// Compares visual information of two links
        /// </summary>
        /// <param name="source">First link's visual information to be compared</param>
        /// <param name="exported">Second link's visual information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareVisual( Link.Visual source,  Link.Visual exported, int indent)
        {
            bool visualNameEqual = (source.name == exported.name);
            linkLog.AppendLine(String.Format("{0}Visual Component Name : {1,6}", Indent(indent + 1), visualNameEqual));
            if (!visualNameEqual)
                return false;

            if (!CompareOrigin( source.origin,  exported.origin, indent))
                return false;

            linkLog.AppendLine(String.Format("{0}Geometry Checks", Indent(indent)));
            if (!CompareGeometry( source.geometry,  exported.geometry, indent+1))
                return false;

            linkLog.AppendLine(String.Format("{0}Material Checks", Indent(indent)));
            if (source.material == null && exported.material == null)
            {
                linkLog.AppendLine(String.Format("{0}Material Nullity Check: {1,6}", Indent(indent), "True"));
                return true;
            }
            else if (source.material != null && exported.material != null)
            {
                linkLog.AppendLine(String.Format("{0}Material Nullity Check: {1,6}", Indent(indent), "True"));
                if (!CompareMaterial(source.material, exported.material, indent + 1))
                {
                    return false;
                }
            }
            else if((source.material == null && exported.material.name == "Default-Material")|| (exported.material == null && source.material?.name == "Default-Material"))
            {
                linkLog.AppendLine(String.Format("{0}Material Nullity Check: {1,6}", Indent(indent), "True"));
                return true;
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Material Nullity Check: {1,6}", Indent(indent), "False"));
                return false;
            }

            return true;
        }

        /// <summary>
        /// Compares geometry information of two visuals
        /// </summary>
        /// <param name="source">First visual's geometry information to be compared</param>
        /// <param name="exported">Second visuals's geometry information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareGeometry( Link.Geometry source,  Link.Geometry exported, int indent)
        {
            if (source.box != null && exported != null)
            {
                bool boxEqual = source.box.size.DoubleDeltaCompare(exported.box.size, .0001);
                linkLog.AppendLine(String.Format("{0}Geometry", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Type:{1,5}", Indent(indent), "Box"));
                linkLog.AppendLine(String.Format("{0}Dimensions Equal: {1,6} ", Indent(indent), boxEqual));
                linkLog.AppendLine(String.Format("{0}Dimensions: Source: {1,5:F3} {2,5:F3} {3,5:F3} Exported: {4,5:F3} {5,5:F3} {6,5:F3}", Indent(indent), source.box.size[0], source.box.size[1], source.box.size[2], exported.box.size[0], exported.box.size[1], exported.box.size[2]));
                if (boxEqual)
                  return true;
                else
                  return false;
            }

            if(source.cylinder != null && exported.cylinder != null)
            {
                bool cylinderEqual = (source.cylinder.radius == exported.cylinder.radius && source.cylinder.length == exported.cylinder.length);
                linkLog.AppendLine(String.Format("{0}Geometry:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Type:{1,5}", Indent(indent), "Cylinder"));
                linkLog.AppendLine(String.Format("{0}Dimensions Equal: {1,6}", Indent(indent), cylinderEqual));
                linkLog.AppendLine(String.Format("{0}Source: Radius: {1,5:F3} Length: {2,5:F3} Exported: Radius: {3,5:F3} Length: {4,5:F3}", Indent(indent), source.cylinder.radius, source.cylinder.length, exported.cylinder.radius, exported.cylinder.length));

                if (cylinderEqual)
                    return true;

                else
                    return false;
            }

            if(source.sphere != null && exported.sphere != null)
            {
                bool sphereEqual = source.sphere.radius.EqualsDelta(exported.sphere.radius, .0001);
                linkLog.AppendLine(String.Format("{0}Geometry:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Type:{1,5}", Indent(indent), "Sphere"));
                linkLog.AppendLine(String.Format("{0}Dimensions Equal: {1,6}", Indent(indent), sphereEqual));
                linkLog.AppendLine(String.Format("{0}Source: Radius: {1,5:F3} Exported: Radius: {1,5:F3}", Indent(indent), source.sphere.radius, exported.sphere.radius));

                if (sphereEqual)
                    return true;  
                else
                    return false;
                
            }

            if (source.mesh != null && exported.mesh != null)
            {
                bool meshNameEqual = (Path.GetFileName(source.mesh.filename) == Path.GetFileName(exported.mesh.filename));
                linkLog.AppendLine(String.Format("{0}Geometry:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Type:{1,5}", Indent(indent), "Mesh"));
                linkLog.AppendLine(String.Format("{0}Name Equal: {1,6}", Indent(indent), meshNameEqual));
                linkLog.AppendLine(String.Format("{0}Name: Source: {1,12} Exported: {2,12}", Indent(indent), source.mesh.filename, exported.mesh.filename));
                if (!meshNameEqual)
                {
                    return false;
                }

                if (source.mesh.scale != null && exported.mesh.scale != null)
                {

                    for (int i = 0; i < source.mesh.scale.Length; i++)
                    {
                        if (source.mesh.scale[i] != exported.mesh.scale[i])
                        {
                            linkLog.AppendLine(String.Format("{0}Scale Equal: {1,6}", Indent(indent), "False"));
                            linkLog.AppendLine(String.Format("{0}Scale: Source: {1,5:F3} {2,5:F3} {3,5:F3} Exported: {4,5:F3} {5,5:F3} {6,5:F3}", Indent(indent),source.mesh.scale[0], source.mesh.scale[1], source.mesh.scale[2], exported.mesh.scale[0], exported.mesh.scale[1], exported.mesh.scale[2]));
                            return false;
                        }
                    }
                    linkLog.AppendLine(String.Format("{0}Scale Equal: {1,6}", Indent(indent), "True"));
                    linkLog.AppendLine(String.Format("{0}Scale:{1,5:F3} {2,5:F3} {3,5:F3}", Indent(indent), source.mesh.scale[0], source.mesh.scale[1], source.mesh.scale[2]));
                }
                else if (source.mesh.scale == null && exported.mesh.scale == null)
                {
                    linkLog.AppendLine(String.Format("{0}Scales Equal : (1,1,1)", Indent(indent)));
                }
                else if ((exported.mesh.scale == null && source.mesh.scale.DoubleDeltaCompare(new double[] { 1, 1, 1 },0)) || (source.mesh.scale == null && exported.mesh.scale.DoubleDeltaCompare(new double[] { 1, 1, 1 },0) ))
                {
                    linkLog.AppendLine(String.Format("{0}Scales Equal : (1,1,1)", Indent(indent)));
                }
                else
                {
                    linkLog.AppendLine(String.Format("{0}Scale Equal: {1,6} ", Indent(indent), "False"));
                    return false;
                }

                return true;

            }

            linkLog.AppendLine(String.Format("{0}No compatible texture shapes found", Indent(indent)));

            return false; 
        }

        /// <summary>
        /// Compares material information of two visuals
        /// </summary>
        /// <param name="source">First visual's material information to be compared</param>
        /// <param name="exported">Second visuals's material information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareMaterial( Link.Visual.Material source,  Link.Visual.Material exported, int indent)
        {
            bool materialNameEqual = source.name == exported.name;
            linkLog.AppendLine(String.Format("{0}Name Equal:{1,6}", Indent(indent), materialNameEqual));
            linkLog.AppendLine(String.Format("{0}Name: Source: {1,12} ", Indent(indent), source.name));
            linkLog.AppendLine(String.Format("{0}Exported: {1,12}", Indent(indent), exported.name));

            if (!materialNameEqual)
            {
                return false;
            }

            if (source.color != null && exported.color != null)
            {

                for (int i = 0; i < 3; i++)
                {
                    if (source.color.rgba[i] != exported.color.rgba[i])
                    {
                        linkLog.AppendLine(String.Format("{0}Colors Equal: {1,6}", Indent(indent), "False"));
                        linkLog.AppendLine(String.Format("{0}RGB Source: {1,5:F3} {2,5:F3} {3,5:F3} ", Indent(indent), source.color.rgba[0], source.color.rgba[1], source.color.rgba[2]));
                        linkLog.AppendLine(String.Format("{0}RGB Exported: {1,5:F3} {2,5:F3} {3,5:F3}", Indent(indent), exported.color.rgba[0], exported.color.rgba[1], exported.color.rgba[2]));
                        return false;
                    }
                }
                linkLog.AppendLine(String.Format("{0}Colors Equal: {1,6}", Indent(indent), "True"));
                linkLog.AppendLine(String.Format("{0}RGB :{1,5:F3} {2,5:F3} {3,5:F3} ", Indent(indent), source.color.rgba[0], source.color.rgba[1], source.color.rgba[2]));
            }
            else if(source.color == null && exported.color == null)
            {
                linkLog.AppendLine(String.Format("{0} Color nullity equality:{1,6}", Indent(indent), "True"));
            }
            else
            {
                linkLog.AppendLine(String.Format("{0} Color nullity equality:{1,6}",Indent(indent), "False"));
                return false;
            }

            if (source.texture != null && exported.texture != null)
            {
                bool textureNameEqual = (source.texture.filename != exported.texture.filename);
                linkLog.AppendLine(String.Format("{0}Name Equal: {1,6}", Indent(indent), textureNameEqual));
                linkLog.AppendLine(String.Format("{0}Name: Source:{1,12}", Indent(indent), source.texture.filename));
                linkLog.AppendLine(String.Format("{0}Exported:{1,12}", Indent(indent), exported.texture.filename));
                if (!textureNameEqual)
                    return false;
            }
            else if(source.texture == null && source.texture == null)
            {
                linkLog.AppendLine(String.Format("{0}Texture nullity equality:{1,6}", Indent(indent), "True"));
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Texture nullity equality:{1,6}",Indent(indent),"False"));
                return false;
            }


            return true;
        }

        /// <summary>
        /// Compares colliison information of two links
        /// </summary>
        /// <param name="source">First links's collision information to be compared</param>
        /// <param name="exported">Second link's collision information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareCollisions( Link.Collision source,  Link.Collision exported, int indent)
        {
            bool colliisonNameEqual = (source.name == exported.name);
            linkLog.AppendLine(String.Format("{0}Collision Name", Indent(indent)));
            linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), colliisonNameEqual));
            linkLog.AppendLine(String.Format("{0}Name: Source: {1,12}", Indent(indent), source.name));
            linkLog.AppendLine(String.Format("{0}Exported: {1,12}", Indent(indent), exported.name));

            if (!colliisonNameEqual)
                return false;

            if (!CompareOrigin( source.origin,  exported.origin, indent))
                return false;
            if (!CompareGeometry( source.geometry,  exported.geometry, indent))
                return false;
            return true;
        }

        /// <summary>
        /// Compares axis information of two links
        /// </summary>
        /// <param name="source">First joint's axis information to be compared</param>
        /// <param name="exported">Second joint's axis information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareAxis(Joint.Axis source, Joint.Axis exported, int indent)
        {
            linkLog.AppendLine(String.Format("{0}Axis Checks", Indent(indent)));
            if (source == null && source == null)
            {
                linkLog.AppendLine(String.Format("{0},Origin Nullity Check: {1,6}", Indent(indent), "True"));
            }
            else if (source == null && exported != null)
            {
                bool axisEqual = exported.xyz.DoubleDeltaCompare(new double[] { 1, 0, 0 }, 0);
                linkLog.AppendLine(String.Format("{0}Axis", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), axisEqual));
                linkLog.AppendLine(String.Format("{0}XYZ Source: NULL ", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}XYZ Exported: ({1,5:F3},{2,5:F3},{3,5:F3}) ", Indent(indent), exported.xyz[0], exported.xyz[1], exported.xyz[2]));

                if (!axisEqual)
                    return false;

            }
            else if (exported == null && source != null)
            {
                bool axisEqual = source.xyz.DoubleDeltaCompare(new double[] { 1, 0, 0 }, 0);
                linkLog.AppendLine(String.Format("{0}Axis", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), axisEqual));
                linkLog.AppendLine(String.Format("{0}XYZ Source: NULL ", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}XYZ Exported: ({1,5:F3},{2,5:F3},{3,5:F3}) ", Indent(indent), exported.xyz[0], exported.xyz[1], exported.xyz[2]));

                if (!axisEqual)
                    return false;
            }
            else
            {
                for (int i = 0; i < 3; i++)
                {
                    if (source.xyz[i] != exported.xyz[i])
                    {
                        linkLog.AppendLine(String.Format("{0}Axis", Indent(indent)));
                        linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "False"));
                        linkLog.AppendLine(String.Format("{0}XYZ Source: ({1,5:F3},{2,5:F3},{3,5:F3}) ", Indent(indent), source.xyz[0], source.xyz[1], source.xyz[2]));
                        linkLog.AppendLine(String.Format("{0}XYZ Exported: ({1,5:F3},{2,5:F3},{3,5:F3}) ", Indent(indent), exported.xyz[0], exported.xyz[1], exported.xyz[2]));
                        return false;
                    }
                }
                linkLog.AppendLine(String.Format("{0}Axis", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "True"));
                linkLog.AppendLine(String.Format("{0}XYZ : ({1,5:F3},{2,5:F3},{3,5:F3}) ", Indent(indent), source.xyz[0], source.xyz[1], source.xyz[2]));
            }

            return true;
        }

        /// <summary>
        /// Compares dynamics information of two links
        /// </summary>
        /// <param name="source">First links's dynamics information to be compared</param>
        /// <param name="exported">Second link's dynamics information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareDynamics(Joint.Dynamics source, Joint.Dynamics exported, int indent)
        {
            linkLog.AppendLine(String.Format("{0}Dynamics Checks", Indent(indent)));
            if (source == null && source == null)
            {
                linkLog.AppendLine(String.Format("{0}Origin Nullity Check: {1,6}", Indent(indent), "True"));
                return true;
            }
            else if (source != null && exported != null)
            {
                if (source.damping != double.NaN && exported.damping != double.NaN)
                {
                    linkLog.AppendLine(String.Format("{0}Damping Nullity Check: {1,6}", Indent(indent), "True"));
                    bool dampingEqual = (source.damping != exported.damping);
                    linkLog.AppendLine(String.Format("{0}Damping:", Indent(indent)));
                    linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), dampingEqual));
                    linkLog.AppendLine(String.Format("{0}Damping Value: Source: {1,12}", Indent(indent), source.damping));
                    linkLog.AppendLine(String.Format("{0}Damping Valye: Exported: {1,12}", Indent(indent), exported.damping));

                    if (dampingEqual)
                        return false;
                 
                }
                else if((source.damping == double.NaN && exported.damping == 0 )|| (exported.damping == double.NaN && source.damping == 0 )|| (source.damping ==double.NaN && exported.damping == double.NaN))
                {
                    linkLog.AppendLine(String.Format("{0}Damping:", Indent(indent)));
                    linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "True"));
                    linkLog.AppendLine(String.Format("{0}Damping Value: 0", Indent(indent)));
                }
                else
                {
                    linkLog.AppendLine(String.Format("{0}Damping Nullity Check: {1,6}", Indent(indent), "False"));
                    return false;
                }


                if (source.friction != double.NaN && exported.friction != double.NaN)
                {
                    linkLog.AppendLine(String.Format("{0}Friction Nullity Check: {1,6}", Indent(indent), "True"));
                    bool frictionEqual = source.friction != exported.friction;
                    linkLog.AppendLine(String.Format("{0}Friction:", Indent(indent)));
                    linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), frictionEqual));
                    linkLog.AppendLine(String.Format("{0}Friction Value: Source: {1,12}", Indent(indent), source.friction));
                    linkLog.AppendLine(String.Format("{0}Friction Value: Exported: {1,12}", Indent(indent), exported.friction));

                    if (frictionEqual)
                        return false;
     
                }
                else if ((source.friction == double.NaN && exported.friction == 0) || (exported.friction == double.NaN && source.friction == 0) || (source.friction == double.NaN && exported.friction == double.NaN))
                {
                    linkLog.AppendLine(String.Format("{0}Friction:", Indent(indent)));
                    linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "True"));
                    linkLog.AppendLine(String.Format("{0}Friction Value: 0", Indent(indent)));
                }
                else
                {
                    linkLog.AppendLine(String.Format("{0}Friction Nullity Check: {1,6}", Indent(indent), "False"));
                    return false;
                }
            }

            else if ((source == null && exported?.damping == 0 && exported?.friction == 0) || (exported == null && source?.damping == 0 && source?.friction == 0))
            {
                linkLog.AppendLine(String.Format("{0}Dynamics Equal: {1,6} ", Indent(indent), "True"));
            }

            else
            {
                linkLog.AppendLine(String.Format("{0}Dynamics Equal: {1,6} ", Indent(indent), "False"));
                return false;
            }
            return true;
        }

        /// <summary>
        /// Compares limit information of two joints
        /// </summary>
        /// <param name="source">First joint's limit information to be compared</param>
        /// <param name="exported">Second joint's limit information to be compared</param>
        /// <param name="indent">Indent level in the log file</param>
        /// <returns></returns>
        private bool CompareLimit(Joint.Limit source, Joint.Limit exported, int indent)
        {
            //Lower
            if((source.lower == double.NaN && exported.lower == 0) || (source.lower == 0 && exported.lower == double.NaN))
            {
                linkLog.AppendLine(String.Format("{0}Lower Limit:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "True"));
                linkLog.AppendLine(String.Format("{0}Lower Limit Value: 0", Indent(indent)));
            }
            else if ((source.lower == double.NaN && exported.lower != 0) || (source.lower != 0 && exported.lower == double.NaN))
            {
                linkLog.AppendLine(String.Format("{0}Lower Limit: {1,6}", Indent(indent), "False"));
                return false;
            }
            else if (!source.lower.EqualsDelta(exported.lower, .05))
            {
                linkLog.AppendLine(String.Format("{0}Lower Limit:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "False"));
                linkLog.AppendLine(String.Format("{0}Lower Limit Value: Source: {1,12}", Indent(indent), source.lower));
                linkLog.AppendLine(String.Format("{0}Lower Limit Value: Exported: {1,12}", Indent(indent), exported.lower));
                return false;
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Lower Limit:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "True"));
                linkLog.AppendLine(String.Format("{0}Lower Limit Value: {1,12}", Indent(indent), source?.lower));
            }

            //Upper
            if ((source.upper == double.NaN && exported.upper == 0) || (source.upper == 0 && exported.upper == double.NaN))
            {
                linkLog.AppendLine(String.Format("{0}Upper Limit:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "True"));
                linkLog.AppendLine(String.Format("{0}Upper Limit Value: 0", Indent(indent)));
            }
            else if ((source.upper == double.NaN && exported.upper != 0) || (source.upper != 0 && exported.upper == double.NaN))
            {
                linkLog.AppendLine(String.Format("{0}Upper Limit: {1,6}", Indent(indent), "False"));
                return false;
            }
            else if (!source.upper.EqualsDelta(exported.upper, .05))
            {
                linkLog.AppendLine(String.Format("{0}Upper Limit:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "False"));
                linkLog.AppendLine(String.Format("{0}Upper Limit Value: Source: {1,12}", Indent(indent), source.upper));
                linkLog.AppendLine(String.Format("{0}Upper Limit Value: Exported: {1,12}", Indent(indent), exported.upper));
                return false;
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Upper Limit:", Indent(indent)));
                linkLog.AppendLine(String.Format("{0}Equal: {1,6}", Indent(indent), "True"));
                linkLog.AppendLine(String.Format("{0}Upper Limit Value: {1,12}", Indent(indent), source?.upper));
            }

            if (!source.effort.EqualsDelta(exported.effort,.05))
            {
                linkLog.AppendLine(String.Format("{0}Effort Equal: {1,6} Effort: Source:{2,5:F3} Exported:{3,5:F3}", Indent(indent), "False", source.effort, exported.effort));
                return false;
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Effort Equal: {1,6} Effort: {2,5:F3}", Indent(indent), "True", source.effort));
            }

            if (!source.velocity.EqualsDelta(exported.velocity, .05))
            {
                linkLog.AppendLine(String.Format("{0}Velocity Limit Equal: {1,6} Velocity: Source:{2,5:F3} Exported:{3,5:F3}", Indent(indent), "False", source.velocity, exported.velocity));
                return false;
            }
            else
            {
                linkLog.AppendLine(String.Format("{0}Velocity Equal: {1,6} Velocity: {2,5:F3}", Indent(indent), "True", source.velocity));
            }

            return true;
        }

        /// <summary>
        /// Returns an a string with number of indents equal to the input parameter
        /// </summary>
        /// <param name="numberofIndents">Number of indents</param>
        /// <returns></returns>
        private string Indent(int numberofIndents)
        {
            string returnString = "";
            for (int i = 0; i < numberofIndents; i++)
                returnString += " ";
            return returnString;
        }

        /// <summary>
        /// Functoion to compare Roll, Pitch Yaw.
        /// It is implemented to take into account equality of angles.
        /// </summary>
        /// <param name="source">First RPY array</param>
        /// <param name="exported">Second RPY array</param>
        /// <param name="delta">Amount difference allowed in comparison</param>
        /// <returns></returns>
        private bool RpyCheck(double[] source, double[] exported, double delta)
        {
            for(int i = 0; i < 3; i++)
            {
                if (source[i].EqualsDelta(exported[i], delta))
                    continue;
                else if (source[i] > 0 || source[i].EqualsDelta(exported[i] + (2 * Mathf.PI), delta))
                    continue;
                else if (source[i] <= 0 || source[i].EqualsDelta(exported[i] - (2 * Mathf.PI), delta))
                    continue;
                else
                    return false;
            }
            return true;
        }
        
    }
}
