  

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{ 
    public static class UrdfCollisionsExtensions
    {
        public static void Create(Transform parent, List<Link.Collision> collisions = null)
        {
            GameObject collisionsObject = new GameObject("Collisions");
            collisionsObject.transform.SetParentAndAlign(parent);
            UrdfCollisions urdfCollisions = collisionsObject.AddComponent<UrdfCollisions>();

            collisionsObject.hideFlags = HideFlags.NotEditable;
            urdfCollisions.hideFlags = HideFlags.None;
            
            if (collisions != null)
            {
                foreach (Link.Collision collision in collisions)
                    UrdfCollisionExtensions.Create(urdfCollisions.transform, collision);
            }
        }
        
        public static List<Link.Collision> ExportCollisionsData(this UrdfCollisions urdfCollisions)
        {
            UrdfCollision[] urdfCollisionsList = urdfCollisions.GetComponentsInChildren<UrdfCollision>();
            return urdfCollisionsList.Select(urdfCollision => urdfCollision.ExportCollisionData()).ToList();
        }
    }
}