  

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfImporterContextMenuItem
    {
        [MenuItem("Assets/Import Robot from URDF")]
        private static void CreateUrdfObject()
        {   
            //Get path to asset, check if it's a urdf file
            string assetPath = AssetDatabase.GetAssetPath(Selection.activeObject);

            if (Path.GetExtension(assetPath)?.ToLower() == ".urdf")
            {
                // Get existing open window or if none, make a new one:
                FileImportMenu window = (FileImportMenu)EditorWindow.GetWindow(typeof(FileImportMenu));
                window.urdfFile = UrdfAssetPathHandler.GetFullAssetPath(assetPath);
                window.minSize = new Vector2(500, 200);
                window.Show();
            }
            else
                EditorUtility.DisplayDialog("URDF Import",
                    "The file you selected was not a URDF file. A robot can only be imported from a valid URDF file.", "Ok");
        }
    }
}