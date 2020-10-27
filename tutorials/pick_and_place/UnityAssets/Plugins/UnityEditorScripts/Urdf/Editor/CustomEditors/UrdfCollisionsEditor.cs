  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfCollisions))]
    class UrdfCollisionsEditor : UnityEditor.Editor
    {
        private UrdfCollisions urdfCollisions;
        private GeometryTypes geometryType;

        public override void OnInspectorGUI()
        {
            urdfCollisions = (UrdfCollisions)target;

            GUILayout.Space(10);
            geometryType = (GeometryTypes)EditorGUILayout.EnumPopup("Type of collision", geometryType);

            if (GUILayout.Button("Add collision"))
                UrdfCollisionExtensions.Create(urdfCollisions.transform, geometryType);
        }
    }
}
