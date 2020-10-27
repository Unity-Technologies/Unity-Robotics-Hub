  

using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfLinkExtensions
    { 
        public static UrdfLink Create(Transform parent, Link link = null, Joint joint = null)
        {
            GameObject linkObject = new GameObject("link");
            linkObject.transform.SetParentAndAlign(parent);
            UrdfLink urdfLink = linkObject.AddComponent<UrdfLink>();
            UrdfVisualsExtensions.Create(linkObject.transform, link?.visuals);
            UrdfCollisionsExtensions.Create(linkObject.transform, link?.collisions);

            if (link != null)
            {
                urdfLink.ImportLinkData(link, joint);
            }
            else
            {
                UrdfInertial.Create(linkObject);
                UnityEditor.EditorGUIUtility.PingObject(linkObject);
            }

            return urdfLink;
        }

        private static void ImportLinkData(this UrdfLink urdfLink, Link link, Joint joint)
        {
            if (link.inertial == null && joint == null)
            {
                urdfLink.IsBaseLink = true;
            }
            urdfLink.gameObject.name = link.name;
            if (joint?.origin != null)
                UrdfOrigin.ImportOriginData(urdfLink.transform, joint.origin);

            if (link.inertial != null)
            {
                UrdfInertial.Create(urdfLink.gameObject, link.inertial);

                if (joint != null)
                    UrdfJoint.Create(urdfLink.gameObject, UrdfJoint.GetJointType(joint.type), joint);
            }
            else if (joint != null)
                UrdfJoint.Create(urdfLink.gameObject, UrdfJoint.GetJointType(joint.type), joint);

            foreach (Joint childJoint in link.joints)
            {
                Link child = childJoint.ChildLink;
                UrdfLinkExtensions.Create(urdfLink.transform, child, childJoint);
            }
        } 
        
        public static Link ExportLinkData(this UrdfLink urdfLink)
        {
            if (urdfLink.transform.localScale != Vector3.one)
                Debug.LogWarning("Only visuals should be scaled. Scale on link \"" + urdfLink.gameObject.name + "\" cannot be saved to the URDF file.", urdfLink.gameObject);
            UrdfInertial urdfInertial = urdfLink.gameObject.GetComponent<UrdfInertial>();
            Link link = new Link(urdfLink.gameObject.name)
            {
                visuals = urdfLink.GetComponentInChildren<UrdfVisuals>().ExportVisualsData(),
                collisions = urdfLink.GetComponentInChildren<UrdfCollisions>().ExportCollisionsData(),
                inertial = urdfInertial == null ? null : urdfInertial.ExportInertialData()
            };
            
            return link;
        }
    }
}