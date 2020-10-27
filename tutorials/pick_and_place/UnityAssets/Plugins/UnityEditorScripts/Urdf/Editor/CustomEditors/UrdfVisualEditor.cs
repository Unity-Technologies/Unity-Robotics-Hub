  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfVisual))]
    class UrdfVisualEditor : UnityEditor.Editor
    {
        private UrdfVisual urdfVisual;

        public override void OnInspectorGUI()
        {
            urdfVisual = (UrdfVisual)target;

            GUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel("Geometry Type");
            EditorGUILayout.LabelField(urdfVisual.geometryType.ToString());
            EditorGUILayout.EndHorizontal();
            
            if (GUILayout.Button("Add collision to match visual"))
            {
                urdfVisual.AddCorrespondingCollision();
            }

            DisplayWarnings();
        }

        private void DisplayWarnings()
        {
            if (!urdfVisual.transform.HasExactlyOneChild())
            {
                GUILayout.Space(5);
                EditorGUILayout.HelpBox("Visual element must have one and only one child Geometry element.", MessageType.Error);
            }
            else if (UrdfGeometry.IsTransformed(urdfVisual.transform.GetChild(0), urdfVisual.geometryType))
            {
                GUILayout.Space(5);
                EditorGUILayout.HelpBox("Changes to the transform of the child Geometry element cannot be exported to URDF. " +
                                        "Make any translation, rotation, or scale changes to this Visual object instead.", MessageType.Error);

                if (GUILayout.Button("Fix transformations"))
                {
                    bool transferRotation = urdfVisual.geometryType != GeometryTypes.Mesh;
                    urdfVisual.transform.MoveChildTransformToParent(transferRotation);
                }
            }
        }
    }
}
