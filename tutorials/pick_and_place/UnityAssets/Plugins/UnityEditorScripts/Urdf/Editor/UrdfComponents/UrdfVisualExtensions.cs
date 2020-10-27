  

using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfVisualExtensions
    {
        public static void Create(Transform parent, GeometryTypes type)
        {
            GameObject visualObject = new GameObject("unnamed");
            visualObject.transform.SetParentAndAlign(parent);
            UrdfVisual urdfVisual = visualObject.AddComponent<UrdfVisual>();

            urdfVisual.geometryType = type;
            UrdfGeometryVisual.Create(visualObject.transform, type);
            UnityEditor.EditorGUIUtility.PingObject(visualObject);
        }

        public static void Create(Transform parent, Link.Visual visual)
        {
            GameObject visualObject = new GameObject(visual.name ?? "unnamed");
            visualObject.transform.SetParentAndAlign(parent);
            UrdfVisual urdfVisual = visualObject.AddComponent<UrdfVisual>();

            urdfVisual.geometryType = UrdfGeometry.GetGeometryType(visual.geometry);
            UrdfGeometryVisual.Create(visualObject.transform, urdfVisual.geometryType, visual.geometry);

            UrdfMaterial.SetUrdfMaterial(visualObject, visual.material);
            UrdfOrigin.ImportOriginData(visualObject.transform, visual.origin);
        }

        public static void AddCorrespondingCollision(this UrdfVisual urdfVisual)
        {
            UrdfCollisions collisions = urdfVisual.GetComponentInParent<UrdfLink>().GetComponentInChildren<UrdfCollisions>();
            UrdfCollisionExtensions.Create(collisions.transform, urdfVisual.geometryType, urdfVisual.transform);
        }

        public static Link.Visual ExportVisualData(this UrdfVisual urdfVisual)
        {
            UrdfGeometry.CheckForUrdfCompatibility(urdfVisual.transform, urdfVisual.geometryType);

            Link.Geometry geometry = UrdfGeometry.ExportGeometryData(urdfVisual.geometryType, urdfVisual.transform);

            Link.Visual.Material material = null;
            if ((geometry.mesh != null )) 
            {
                material = UrdfMaterial.ExportMaterialData(urdfVisual.GetComponentInChildren<MeshRenderer>().sharedMaterial);
            }
            string visualName = urdfVisual.name == "unnamed" ? null : urdfVisual.name;

            return new Link.Visual(geometry, visualName, UrdfOrigin.ExportOriginData(urdfVisual.transform), material);
        }
    }
}