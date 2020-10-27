  

using System.IO;
using System;
using UnityEngine;
using UnityEditor;

namespace RosSharp.Urdf.Editor
{
    public static class LocateAssetHandler
    {
        public static T FindUrdfAsset<T>(string urdfFileName) where T : UnityEngine.Object
        {
            string fileAssetPath = UrdfAssetPathHandler.GetRelativeAssetPathFromUrdfPath(urdfFileName);
            T assetObject = AssetDatabase.LoadAssetAtPath<T>(fileAssetPath);

            if (assetObject != null)
                return assetObject;

            //If asset was not found, let user choose whether to search for
            //or ignore the missing asset.
            string invalidPath = fileAssetPath ?? urdfFileName;
            int option = EditorUtility.DisplayDialogComplex("Urdf Importer: Asset Not Found",
                "Current root folder: " + UrdfAssetPathHandler.GetPackageRoot() +
                "\n\nExpected asset path: " + invalidPath,
                "Locate Asset",
                "Ignore Missing Asset",
                "Locate Root Folder");

            switch (option)
            {
                case 0:
                    fileAssetPath = LocateAssetFile(invalidPath);
                    break;
                case 1: break;
                case 2:
                    fileAssetPath = LocateRootAssetFolder<T>(urdfFileName);
                    break;
            }

            assetObject = (T) AssetDatabase.LoadAssetAtPath(fileAssetPath, typeof(T));
            if (assetObject != null)
                return assetObject;

            ChooseFailureOption(urdfFileName);
            return null;
        }

        private static string LocateRootAssetFolder<T>(string urdfFileName) where T : UnityEngine.Object
        {
            string newAssetPath = EditorUtility.OpenFolderPanel(
                "Locate package root folder",
                Path.Combine(Path.GetDirectoryName(Application.dataPath), "Assets"),
                "");

            if (UrdfAssetPathHandler.IsValidAssetPath(newAssetPath))
                UrdfAssetPathHandler.SetPackageRoot(newAssetPath, true);
            else 
                Debug.LogWarning("Selected package root " + newAssetPath + " is not within the Assets folder.");

            return UrdfAssetPathHandler.GetRelativeAssetPathFromUrdfPath(urdfFileName);
        }

        private static string LocateAssetFile(string invalidPath)
        {
            string fileExtension = Path.GetExtension(invalidPath)?.Replace(".", "");

            string newPath = EditorUtility.OpenFilePanel(
                "Couldn't find asset at " + invalidPath + ". Select correct file.",
                UrdfAssetPathHandler.GetPackageRoot(),
                fileExtension);

            return UrdfAssetPathHandler.GetRelativeAssetPath(newPath);
        }

        private static void ChooseFailureOption(string urdfFilePath)
        {
            if (!EditorUtility.DisplayDialog(
                "Urdf Importer: Missing Asset",
                "Missing asset " + Path.GetFileName(urdfFilePath) +
                " was ignored or could not be found.\n\nContinue URDF Import?",
                "Yes",
                "No"))
            {
                throw new InterruptedUrdfImportException("User cancelled URDF import. Model may be incomplete.");
            }
        }
        
        private class InterruptedUrdfImportException : Exception
        {
            public InterruptedUrdfImportException(string message) : base(message)
            {
            }
        }
    }
}
