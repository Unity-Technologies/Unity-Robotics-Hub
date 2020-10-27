  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfCollision))]
    class UrdfCollisionEditor : UnityEditor.Editor
    {
        private UrdfCollision urdfCollision;

        public override void OnInspectorGUI()
        {
            urdfCollision = (UrdfCollision)target;

            GUILayout.Space(5);

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.PrefixLabel("Geometry Type");
            EditorGUILayout.LabelField(urdfCollision.geometryType.ToString());
            EditorGUILayout.EndHorizontal();

            DisplayWarnings();
        }

        private void DisplayWarnings()
        {
            if (!urdfCollision.transform.HasExactlyOneChild())
            {
                GUILayout.Space(5);
                EditorGUILayout.HelpBox("Visual element must have one and only one child Geometry element.", MessageType.Error);
            }
            else if (UrdfGeometry.IsTransformed(urdfCollision.transform.GetChild(0), urdfCollision.geometryType))
            {
                GUILayout.BeginVertical("HelpBox");
                EditorGUILayout.HelpBox("Changes to the transform of the child Geometry element cannot be exported to URDF. " +
                                        "Make any translation, rotation, or scale changes to this Visual object instead.", MessageType.Error);

                if (GUILayout.Button("Fix transformations"))
                {
                    //Only transfer rotation if geometry is not a mesh
                    bool transferRotation = urdfCollision.geometryType != GeometryTypes.Mesh;
                    urdfCollision.transform.MoveChildTransformToParent(transferRotation);
                }
                GUILayout.EndVertical();
            }
        }
    }
}
