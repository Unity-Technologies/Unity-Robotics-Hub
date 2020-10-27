  

using UnityEditor;

namespace RosSharp.Urdf.Editor
{
    public static class UrdfRobotCreatorMenuItem
    {
        [MenuItem("GameObject/3D Object/URDF Model (new)")]
        private static void CreateUrdfObject()
        {

            UrdfRobotExtensions.Create();
        }
    }
}