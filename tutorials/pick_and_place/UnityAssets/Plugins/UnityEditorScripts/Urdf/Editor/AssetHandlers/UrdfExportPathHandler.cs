  
using System.IO;

namespace RosSharp.Urdf
{
    public static class UrdfExportPathHandler
    {
        //absolute path to export folder
        private static string exportRoot;
        //Relative to export root folder
        private static string subfolder;

        private const string MeshFolderName = "meshes";
        private const string ResourceFolderName = "resources";

        public static void SetExportPath(string root, string subRoot = "")
        {
            exportRoot = root;
            subfolder = subRoot;

            Directory.CreateDirectory(GetExportDestination());
            Directory.CreateDirectory(Path.Combine(GetExportDestination(), MeshFolderName));
            Directory.CreateDirectory(Path.Combine(GetExportDestination(), ResourceFolderName));
        }

        #region GetExportPaths
        public static string GetExportDestination()
        {
            return subfolder == null ? exportRoot : Path.Combine(exportRoot, subfolder).SetSeparatorChar();
        }
        
        //Returns an absolute path to the export destination for the mesh
        //meshFileName includes the file extension
        public static string GetNewMeshPath(string meshFileName)
        {
            return Path.Combine(exportRoot, subfolder, MeshFolderName, meshFileName).SetSeparatorChar();
        }

        //Returns an absolute path to the new resource
        public static string GetNewResourcePath(string resourceFileName)
        {
            return Path.Combine(exportRoot, subfolder, ResourceFolderName, resourceFileName)
                .SetSeparatorChar();
        }

        public static string GetPackagePathForMesh(string meshPath)
        {
            //All package paths should use forward slashes
            return Path.Combine("package://", subfolder, MeshFolderName, Path.GetFileName(meshPath)).Replace("\\", "/");
        }

        public static string GetPackagePathForResource(string resourcePath)
        {
            //All package paths should use forward slashes
            return Path.Combine("package://", subfolder, ResourceFolderName, Path.GetFileName(resourcePath)).Replace("\\", "/");
        }
        #endregion

        public static void Clear()
        {
            exportRoot = "";
            subfolder = "";
        }
    }

}