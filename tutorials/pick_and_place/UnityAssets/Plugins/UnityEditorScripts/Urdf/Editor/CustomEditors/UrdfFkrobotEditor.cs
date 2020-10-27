using UnityEditor;
using UnityEngine;
using RosSharp.Control;

namespace RosSharp.Urdf.Editor
{
    [CustomEditor(typeof(FKRobot), true)]
    public class UrdfFkrobotEditor : UnityEditor.Editor
    {

        private FKRobot fkrobot;

        private SerializedProperty currentAngles;
        private SerializedProperty rotationMatrix;

        private void OnEnable()
        {
            currentAngles = serializedObject.FindProperty("currentAngles");
            rotationMatrix = serializedObject.FindProperty("endEffectorPosition");
        }
        public override void OnInspectorGUI()
        {
            fkrobot = (FKRobot)target;

            serializedObject.Update();
            EditorGUILayout.PropertyField(currentAngles);
            EditorGUILayout.PropertyField(rotationMatrix);
            serializedObject.ApplyModifiedProperties();

            GUILayout.Space(5);
            if (GUILayout.Button("Add DH Parameters"))
            {
                AddDhParameterWindow window = (AddDhParameterWindow)EditorWindow.GetWindow(typeof(AddDhParameterWindow));
                window.script = fkrobot;
                window.minSize = new Vector2(500, 200);
                window.GetEditorPrefs();
                window.Show();
            }
        }


    }
}
