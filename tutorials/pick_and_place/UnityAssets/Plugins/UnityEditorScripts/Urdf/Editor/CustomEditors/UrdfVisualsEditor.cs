  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfVisuals))]
    class UrdfVisualsEditor : UnityEditor.Editor
    {
        private UrdfVisuals urdfVisuals;
        private GeometryTypes geometryType = GeometryTypes.Box;

        public override void OnInspectorGUI()
        {
            urdfVisuals = (UrdfVisuals)target;

            GUILayout.Space(10);
            geometryType = (GeometryTypes)EditorGUILayout.EnumPopup("Type of visual", geometryType);
            
            EditorGUILayout.BeginHorizontal();
            if (GUILayout.Button("Add visual"))
                UrdfVisualExtensions.Create(urdfVisuals.transform, geometryType);
            EditorGUILayout.EndHorizontal();
        }
    }
}
