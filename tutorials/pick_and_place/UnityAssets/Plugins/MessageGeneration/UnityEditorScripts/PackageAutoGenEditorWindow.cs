using System.Collections.Generic;
using System.IO;
using System.Linq;
using UnityEditor;
using UnityEngine;

 namespace RosMessageGeneration
{
    public abstract class PackageAutoGenEditorWindow : EditorWindow
    {
        [SerializeField]
        private static string lastPackageDirectory = string.Empty;
        [SerializeField]
        private static string lastOutputDirectory = string.Empty;

        private string inPkgPath = "";
        private string rosPackageName = "";
        private string outPkgPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosMessages");

        protected abstract string GenerationType { get; }
        protected abstract string FileExtension { get; }

        protected virtual void OnGUI()
        {
            GUILayout.Label("Package " + GenerationType + " auto generation", EditorStyles.boldLabel);

            EditorGUILayout.BeginHorizontal();
            inPkgPath = EditorGUILayout.TextField("Input Package Path", inPkgPath);
            if (GUILayout.Button("Browse Package...", GUILayout.Width(150)))
            {
                inPkgPath = EditorUtility.OpenFolderPanel("Select Package...", lastPackageDirectory, "");
                if (!inPkgPath.Equals(""))
                {
                    rosPackageName = inPkgPath.Split('/').Last();
                }
            }
            EditorGUILayout.EndHorizontal();

            rosPackageName = EditorGUILayout.TextField("ROS Package Name:", rosPackageName);

            EditorGUILayout.BeginHorizontal();
            outPkgPath = EditorGUILayout.TextField("Output Location", outPkgPath);
            if (GUILayout.Button("Select Folder...", GUILayout.Width(150)))
            {
                outPkgPath = EditorUtility.OpenFolderPanel("Select Folder...", lastOutputDirectory, "");
            }
            EditorGUILayout.EndHorizontal();

            if (GUILayout.Button("GENERATE!"))
            {
                if (inPkgPath.Equals(""))
                {
                    EditorUtility.DisplayDialog(
                        title: "Error",
                        message: "Empty input package path!\nPlease specify input package",
                        ok: "Bricks without straw");
                }
                else
                {
                    lastPackageDirectory = inPkgPath;
                    lastOutputDirectory = outPkgPath;
                    try
                    {
                        string[] files = Directory.GetFiles(Path.Combine(inPkgPath, FileExtension), "*." + FileExtension);
                        if (files.Length == 0)
                        {
                            EditorUtility.DisplayDialog(
                                title: "No action files found!",
                                message: "No action files found!",
                                ok: "Bricks without straw");
                            Reset();
                        }
                        else
                        {
                            // Keep a list of warnings
                            List<string> warnings = new List<string>();
                            for (int i = 0; i < files.Length; i++)
                            {
                                string file = files[i];
                                EditorUtility.DisplayProgressBar(
                                    "Working...(" + (i + 1) + "/" + files.Length + ")",
                                    "Parsing " + file,
                                    (i + 1) / (float)files.Length);
                                try
                                {
                                    warnings.AddRange(Generate(file, outPkgPath, rosPackageName));
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
                            // Done
                            EditorUtility.ClearProgressBar();
                            AssetDatabase.Refresh();
                            if (warnings.Count > 0)
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Complete",
                                    message: "Output at: " + outPkgPath + "\nYou have " + warnings.Count + " warning(s)",
                                    ok: "I like to live dangerously");
                                foreach (string w in warnings)
                                {
                                    Debug.LogWarning(w);
                                }
                            }
                            else
                            {
                                EditorUtility.DisplayDialog(
                                    title: "Code Generation Complete",
                                    message: "Output at: " + outPkgPath,
                                    ok: "Thank you!");
                            }
                            Reset();
                        }
                    }
                    catch (DirectoryNotFoundException e)
                    {
                        EditorUtility.DisplayDialog(
                            title: "Message Folder not found",
                            message: e.Message,
                            ok: "Bricks without straw");
                        Reset();
                    }
                }
            }
        }

        private void OnInspectorUpdate()
        {
            Repaint();
        }

        private void Reset()
        {
            inPkgPath = "";
            rosPackageName = "";
            outPkgPath = Path.Combine(System.Environment.CurrentDirectory, "Assets", "RosMessages");
        }

        protected abstract List<string> Generate(string inPath, string outPath, string rosPackageName = "");
    }
}
