using System.IO;
using System;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public class FileImportMenu : EditorWindow
    {
        public string urdfFile;
        public axisType choosenAxis = axisType.yAxis;
        private static string[] windowOptions = { };

        private void OnGUI()
        {
            //Styles definitions
            GUIStyle titleStyle = new GUIStyle(EditorStyles.boldLabel)
            {
                alignment = TextAnchor.MiddleCenter,
                fontSize = 13
            };
            GUIStyle buttonStyle = new GUIStyle(EditorStyles.miniButtonRight) { fixedWidth = 75 };

            //Window title
            GUILayout.Space(10);
            GUILayout.Label("Select Axis Type", titleStyle);

            //Select Axis to be imported
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal();
            choosenAxis = (axisType)EditorGUILayout.EnumPopup(
                "Select Axis Type" , choosenAxis);
            EditorGUILayout.EndHorizontal();


            //Import Robot button
            GUILayout.Space(10);
            if (GUILayout.Button("Import URDF"))
            {
                if (urdfFile != "")
                    UrdfRobotExtensions.Create(urdfFile,choosenAxis);
                Close();
            }

        }

    }
}