using System.IO;
using UnityEditor;
using UnityEngine;
using RosSharp.Urdf.Testing;

namespace RosSharp.Urdf.Editor
{
    public class CompareURDF : EditorWindow
    {
        public UrdfComparator comparator;
        public string originalFile = "";
        public string exportedFile = "";
        public string logFileLocation = ""; // Defaults to the location of exported file location
        private static string[] windowOptions = { "originalFile","exportedFile","logFileLocation" };

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
            GUILayout.Label("Compare URDF files", titleStyle);

            //Select imported URDF file
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal();
            originalFile = EditorGUILayout.TextField(
                new GUIContent("Source URDF file : ", "The original robot URDF file"),
                originalFile);
            if (GUILayout.Button("Select", buttonStyle))
            {
                originalFile = EditorUtility.OpenFilePanel("Select source URDF file", originalFile, "");
            }

            EditorGUILayout.EndHorizontal();

            //Select Exported File

            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal();
            exportedFile = EditorGUILayout.TextField(
                new GUIContent("Exported URDF file : ", "The exported robot URDF file"),
                exportedFile);
            if (GUILayout.Button("Select", buttonStyle))
            {
                exportedFile = EditorUtility.OpenFilePanel("Select exported URDF file", exportedFile, "");
                logFileLocation = Path.GetDirectoryName(exportedFile);
            }

            EditorGUILayout.EndHorizontal();

            //Log File Location
            GUILayout.Space(5);
            EditorGUILayout.BeginHorizontal();
            logFileLocation = EditorGUILayout.TextField(
                new GUIContent("Log File Save Location : ", "Log File Location "),
                logFileLocation);
            if (GUILayout.Button("Select", buttonStyle))
            {
                logFileLocation = EditorUtility.OpenFolderPanel("Log File Save Location Folder", exportedFile, "");
            }

            EditorGUILayout.EndHorizontal();

            //Export Robot button
            bool flag;
            GUILayout.Space(10);
            if (GUILayout.Button("Comapre URDF Files"))
            {
                if (!FileCheck(originalFile) || !FileCheck(exportedFile))
                    EditorUtility.DisplayDialog("File Error",
                        "File(s) selected is not valid", "Ok");
                else
                {
                    comparator = new UrdfComparator(originalFile, exportedFile, logFileLocation);
                    flag = comparator.Compare();
                    Close();
                }
            }

        }

        /// <summary>
        /// Check if the file selected is a valid file by checking if the file selected is of the form .urdf.
        /// </summary>
        /// <param name="filePath">Path of the file selected</param>
        /// <returns></returns>
        private bool FileCheck(string filePath)
        {
            if(filePath == " ")
            {
                return false;
            }

            if (!File.Exists(filePath))
            {
                return false;
            }

            if (filePath.Substring(filePath.Length - 4) != "urdf")
            {
                return false;
            }

            return true;
        }

        private void OnDestroy()
        {
            SetEditorPrefs();
        }

        private void SetEditorPrefs()
        {
            EditorPrefs.SetString(windowOptions[0], originalFile);
            EditorPrefs.SetString(windowOptions[1], exportedFile);
            EditorPrefs.SetString(windowOptions[2], logFileLocation);
        }

        public void GetEditorPrefs()
        {
            originalFile = EditorPrefs.HasKey(windowOptions[0]) ?
                EditorPrefs.GetString(windowOptions[0]) : "";
            exportedFile = EditorPrefs.HasKey(windowOptions[1]) ?
                EditorPrefs.GetString(windowOptions[1]) : "";
            logFileLocation = EditorPrefs.HasKey(windowOptions[2]) ?
                EditorPrefs.GetString(windowOptions[2]) : "";
        }

    }
}