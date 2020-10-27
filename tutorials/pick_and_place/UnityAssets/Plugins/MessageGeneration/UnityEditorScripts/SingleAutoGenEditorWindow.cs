using System.Collections.Generic;
using System.IO;
using UnityEditor;
using UnityEngine;

 namespace RosMessageGeneration
{
    public abstract class SingleAutoGenEditorWindow : EditorWindow
    {
        [SerializeField]
        private static string lastFileDirectory = string.Empty;
        [SerializeField]
        private static string lastOutputDirectory = string.Empty;

        private string inFilePath = "";
        private string outFilePath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosMessages");
        private string rosPackageName = "";

        protected abstract string GenerationType { get; }
        protected abstract string FileExtension { get; }

        protected virtual void OnGUI()
        {
            GUILayout.Label("Single " + GenerationType + " auto generation", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            inFilePath = EditorGUILayout.TextField("Input File Path", inFilePath);
            if (GUILayout.Button("Browse File...", GUILayout.Width(120)))
            {
                inFilePath = EditorUtility.OpenFilePanel("Select " + GenerationType + " File...", lastFileDirectory, FileExtension);
                if (!inFilePath.Equals(""))
                {
                    string[] directoryLevels = inFilePath.Split('/');
                    rosPackageName = directoryLevels[directoryLevels.Length - 3];
                }
            }
            EditorGUILayout.EndHorizontal();

            rosPackageName = EditorGUILayout.TextField("ROS Package Name:", rosPackageName);

            EditorGUILayout.BeginHorizontal();
            outFilePath = EditorGUILayout.TextField("Output File Location", outFilePath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(120)))
            {
                outFilePath = EditorUtility.OpenFolderPanel("Select Folder...", lastOutputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("GENERATE!"))
            {
                if (inFilePath.Equals(""))
                {
                    EditorUtility.DisplayDialog(
                        title: "Error",
                        message: "Empty input file path!\nPlease specify input file",
                        ok: "Bricks without straw");
                }
                else
                {
                    lastFileDirectory = inFilePath;
                    lastOutputDirectory = outFilePath;
                    try
                    {
                        List<string> warnings = Generate(inFilePath, outFilePath, rosPackageName);
                        AssetDatabase.Refresh();
                        if (warnings.Count == 0)
                        {
                            EditorUtility.DisplayDialog(
                                title: "Code Generation Complete",
                                message: "Output at: " + outFilePath,
                                ok: "Thank you!");
                        }
                        else
                        {
                            foreach (string w in warnings)
                            {
                                Debug.LogWarning(w);
                            }
                            EditorUtility.DisplayDialog(
                                title: "Code Generation Complete",
                                message: "Output at: " + outFilePath + "\nYou have " + warnings.Count + " warning(s)",
                                ok: "I like to live dangerously");
                        }
                    }
                    catch (MessageTokenizerException e)
                    {
                        Debug.LogError(e.ToString() + e.Message);
                        EditorUtility.DisplayDialog(
                            title: "Message Tokenizer Exception",
                            message: e.Message,
                            ok: "Wait. That's illegal");
                    }
                    catch (MessageParserException e)
                    {
                        Debug.LogError(e.ToString() + e.Message);
                        EditorUtility.DisplayDialog(
                            title: "Message Parser Exception",
                            message: e.Message,
                            ok: "Sorry but you can't ignore errors.");
                    }
                }
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        protected abstract List<string> Generate(string inPath, string outPath, string rosPackageName = "");
    }
}
