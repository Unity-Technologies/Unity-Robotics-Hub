  

using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfCollisionExtensions
    {
        public static void Create(Transform parent, GeometryTypes type, Transform visualToCopy = null)
        {
            GameObject collisionObject = new GameObject("unnamed");
            collisionObject.transform.SetParentAndAlign(parent);

            UrdfCollision urdfCollision = collisionObject.AddComponent<UrdfCollision>();
            urdfCollision.geometryType = type;

            if (visualToCopy != null)
            {
                if (urdfCollision.geometryType == GeometryTypes.Mesh)
                    UrdfGeometryCollision.CreateMatchingMeshCollision(collisionObject.transform, visualToCopy);
                else
                    UrdfGeometryCollision.Create(collisionObject.transform, type);

                //copy transform values from corresponding UrdfVisual
                collisionObject.transform.position = visualToCopy.position;
                collisionObject.transform.localScale = visualToCopy.localScale;
                collisionObject.transform.rotation = visualToCopy.rotation;
            }
            else
                UrdfGeometryCollision.Create(collisionObject.transform, type);

            UnityEditor.EditorGUIUtility.PingObject(collisionObject);
        }

        public static void Create(Transform parent, Link.Collision collision)
        {
            GameObject collisionObject = new GameObject("unnamed");
            collisionObject.transform.SetParentAndAlign(parent);
            UrdfCollision urdfCollision = collisionObject.AddComponent<UrdfCollision>();
            urdfCollision.geometryType = UrdfGeometry.GetGeometryType(collision.geometry);
            UrdfGeometryCollision.Create(collisionObject.transform, urdfCollision.geometryType, collision.geometry);
            UrdfOrigin.ImportOriginData(collisionObject.transform, collision.origin);
        }
    
        public static Link.Collision ExportCollisionData(this UrdfCollision urdfCollision)
        {
            UrdfGeometry.CheckForUrdfCompatibility(urdfCollision.transform, urdfCollision.geometryType);

            Link.Geometry geometry = UrdfGeometry.ExportGeometryData(urdfCollision.geometryType, urdfCollision.transform, true);
            string collisionName = urdfCollision.name == "unnamed" ? null : urdfCollision.name;

            return new Link.Collision(geometry, collisionName, UrdfOrigin.ExportOriginData(urdfCollision.transform));
        }
    }
}