  

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public class UrdfImporterMenuItem
    {
        [MenuItem("GameObject/3D Object/URDF Model (import)")]
        private static void CreateUrdfObject()
        {
            string urdfFile = EditorUtility.OpenFilePanel(
                "Import local URDF",
                Path.Combine(Path.GetDirectoryName(Application.dataPath),"Assets"),
                "urdf");
            // Get existing open window or if none, make a new one:
            FileImportMenu window = (FileImportMenu)EditorWindow.GetWindow(typeof(FileImportMenu));
            window.urdfFile = urdfFile;
            window.minSize = new Vector2(500, 200);
            window.Show();
        }
    }
}