  

using System.Collections.Generic;
using System.Linq;
using UnityEngine;

namespace RosSharp.Urdf
{
    public class UrdfPlugins : MonoBehaviour
    {
        public static void Create(Transform robot, List<Plugin> plugins = null)
        {
            GameObject pluginsObject = new GameObject("Plugins");
            pluginsObject.transform.SetParentAndAlign(robot);
            pluginsObject.AddComponent<UrdfPlugins>();

            if (plugins == null) return;

            foreach (var plugin in plugins)
                UrdfPlugin.Create(pluginsObject.transform, plugin);
        }

        public List<Plugin> ExportPluginsData()
        {
            return GetComponents<UrdfPlugin>()
                .Select(urdfPlugin => urdfPlugin.ExportPluginData())
                .Where(plugin => plugin != null)
                .ToList();
        }
    }

}