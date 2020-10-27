  

using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(UrdfLink))]
    public class UrdfLinkEditor : UnityEditor.Editor
    {
        private UrdfLink urdfLink;
        private UrdfJoint.JointTypes jointType = UrdfJoint.JointTypes.Fixed;

        public override void OnInspectorGUI()
        {
            urdfLink = (UrdfLink) target;

            GUILayout.Space(5);
            urdfLink.IsBaseLink = EditorGUILayout.Toggle("Is Base Link", urdfLink.IsBaseLink);
            GUILayout.Space(5);

            EditorGUILayout.BeginVertical("HelpBox");
            jointType = (UrdfJoint.JointTypes) EditorGUILayout.EnumPopup(
                "Child Joint Type", jointType);

            if (GUILayout.Button("Add child link (with joint)"))
            {
                UrdfLink childLink = UrdfLinkExtensions.Create(urdfLink.transform);
                UrdfJoint.Create(childLink.gameObject, jointType);
            }
            EditorGUILayout.EndVertical();
        }
    }
}
