  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfInertial))]
    public class UrdfInertialEditor : UnityEditor.Editor
    {
        private Vector3 testVector;

        public override void OnInspectorGUI()
        {
            UrdfInertial urdfInertial = (UrdfInertial) target;

            GUILayout.Space(5);
            urdfInertial.displayInertiaGizmo =
                EditorGUILayout.ToggleLeft("Display Inertia Gizmo", urdfInertial.displayInertiaGizmo);
            GUILayout.Space(5);

            bool newValue = EditorGUILayout.BeginToggleGroup("Use URDF Data", urdfInertial.useUrdfData);
            EditorGUILayout.Vector3Field("URDF Center of Mass", urdfInertial.centerOfMass);
            EditorGUILayout.Vector3Field("URDF Inertia Tensor", urdfInertial.inertiaTensor);
            EditorGUILayout.Vector3Field("URDF Inertia Tensor Rotation",
                urdfInertial.inertiaTensorRotation.eulerAngles);
            EditorGUILayout.EndToggleGroup();

            if (newValue != urdfInertial.useUrdfData)
            {
                urdfInertial.useUrdfData = newValue;
                urdfInertial.UpdateLinkData();
            }
        }
    }
}