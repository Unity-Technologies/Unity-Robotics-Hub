  
using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfVisualsExtensions
    {
        public static void Create(Transform parent, List<Link.Visual> visuals = null)
        {
            GameObject visualsObject = new GameObject("Visuals");
            visualsObject.transform.SetParentAndAlign(parent);
            UrdfVisuals urdfVisuals = visualsObject.AddComponent<UrdfVisuals>();

            visualsObject.hideFlags = HideFlags.NotEditable;
            urdfVisuals.hideFlags = HideFlags.None;

            if (visuals != null)
            {
                foreach (Link.Visual visual in visuals)
                    UrdfVisualExtensions.Create(urdfVisuals.transform, visual);
            }
        }

        public static List<Link.Visual> ExportVisualsData(this UrdfVisuals urdfVisuals)
        {
            UrdfVisual[] urdfVisualsList = urdfVisuals.GetComponentsInChildren<UrdfVisual>();

            return urdfVisualsList.Select(urdfCollision => urdfCollision.ExportVisualData()).ToList();
        }
    }
}

