  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfPlugins))]
    class UrdfPluginsEditor : UnityEditor.Editor
    {
        private GeometryTypes geometryType;

        public override void OnInspectorGUI()
        {
            UrdfPlugins urdfPlugins = (UrdfPlugins)target;

            GUILayout.Space(8);

            if (GUILayout.Button("Add Plugin"))
                UrdfPlugin.Create(urdfPlugins.transform);
        }
    }
}
