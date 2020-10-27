using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

 namespace RosMessageGeneration
{
    public class PackageSrvAutoGenEditorWindow : PackageAutoGenEditorWindow
    {
        protected override string GenerationType
        {
            get { return "service"; }
        }

        protected override string FileExtension
        {
            get { return "srv"; }
        }

        [MenuItem("RosMessageGeneration/Auto Generate Services/Package Services...", false, 11)]
        private static void OpenWindow()
        {
            PackageSrvAutoGenEditorWindow window = GetWindow<PackageSrvAutoGenEditorWindow>(false, "Service Auto Generation", true);
            window.minSize = new Vector2(750, 100);
            window.maxSize = new Vector2(750, 100);
            window.Show();
        }

        protected override List<string> Generate(string inPath, string outPath, string rosPackageName = "")
        {
            return ServiceAutoGen.GenerateSingleService(inPath, outPath, rosPackageName);
        }
    }
}