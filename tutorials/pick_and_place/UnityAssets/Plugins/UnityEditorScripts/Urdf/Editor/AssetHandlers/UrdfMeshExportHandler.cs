  

using System.IO;
using UnityEditor;
using UnityEngine;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfMeshExportHandler
    {
        public static string CopyOrCreateMesh(GameObject geometryObject, bool isCollisionGeometry)
        {
            string prefabPath = GetPrefabPath(geometryObject);

            bool foundExistingColladaOrStl = false;
            if (prefabPath != null && prefabPath != "")
            {
                if (Path.GetExtension(prefabPath).ToLower() == ".dae")
                    foundExistingColladaOrStl = true;
                else //Find STL file that corresponds to the prefab, if it already exists
                {
                    string[] foldersToSearch = {Path.GetDirectoryName(prefabPath)};
                    string prefabName = Path.GetFileNameWithoutExtension(prefabPath);

                    foreach (string guid2 in AssetDatabase.FindAssets(prefabName, foldersToSearch))
                    {
                        string possiblePath = AssetDatabase.GUIDToAssetPath(guid2);
                        if (possiblePath.ToLower().Contains(".stl"))
                        {
                            prefabPath = possiblePath;
                            foundExistingColladaOrStl = true;
                            break;
                        }
                    }
                }
            }
            
            if (foundExistingColladaOrStl)
                return CopyMeshToExportDestination(prefabPath);

            return CreateNewStlFile(geometryObject, isCollisionGeometry);
        }

        private static string CopyMeshToExportDestination(string prefabPath)
        {
            string newPrefabPath = UrdfExportPathHandler.GetNewMeshPath(Path.GetFileName(prefabPath));

            if (Path.GetExtension(prefabPath)?.ToLower() == ".dae")
                CopyDaeTextureToExportDestination(prefabPath, Path.GetDirectoryName(newPrefabPath));

            prefabPath = UrdfAssetPathHandler.GetFullAssetPath(prefabPath);
            
            CopyFileToNewLocation(prefabPath, newPrefabPath);

            return newPrefabPath;
        }

        private static void CopyDaeTextureToExportDestination(string prefabPath, string newFolderLocation)
        {
            //Get material from Collada prefab
            Material material = AssetDatabase.LoadAssetAtPath<Material>(prefabPath);
            if (material.mainTexture == null) return;
            
            //Get relative subfolder where texture is, compared to the DAE file.
            string commonFolder = Path.GetDirectoryName(prefabPath).SetSeparatorChar();
            string texturePath = AssetDatabase.GetAssetPath(material.mainTexture).SetSeparatorChar();
            string relativeLocation = "";
            if (texturePath.Contains(commonFolder))
                relativeLocation = texturePath.Substring(commonFolder.Length + 1);
            string newTexturePath = Path.Combine(newFolderLocation, relativeLocation);

            Directory.CreateDirectory(Path.GetDirectoryName(newTexturePath));
            
            CopyFileToNewLocation(UrdfAssetPathHandler.GetFullAssetPath(texturePath), newTexturePath);
        }

        private static void CopyFileToNewLocation(string oldPath, string newPath)
        {
            if (oldPath != newPath)
                File.Copy(oldPath, newPath, true);
        }

        private static string CreateNewStlFile(GameObject geometryObject, bool isCollisionGeometry)
        {
            Debug.Log("Did not find an existing STL or DAE file for Geometry Mesh "
                      + geometryObject.name + ". Exporting a new STL file.", geometryObject);

            string newMeshPath = UrdfExportPathHandler.GetNewMeshPath(geometryObject.name + ".stl");

            StlExporter stlExporter = new StlExporter(newMeshPath, geometryObject, isCollisionGeometry);
            if (!stlExporter.Export())
                Debug.LogWarning("Mesh export for geometry " + geometryObject.name + " failed.", geometryObject);

            return newMeshPath;
        }

        private static string GetPrefabPath(GameObject gameObject)
        {
            return AssetDatabase.GetAssetPath(PrefabUtility.GetCorrespondingObjectFromSource(gameObject));
        } 
    }
}