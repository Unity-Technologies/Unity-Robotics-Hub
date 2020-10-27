using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

 namespace RosMessageGeneration
{
    public class PackageMsgAutoGenEditorWindow : PackageAutoGenEditorWindow
    {
        protected override string GenerationType
        {
            get { return "message"; }
        }

        protected override string FileExtension
        {
            get { return "msg"; }
        }

        [MenuItem("RosMessageGeneration/Auto Generate Messages/Package Messages...", false, 1)]
        private static void OpenWindow()
        {
            PackageMsgAutoGenEditorWindow window = GetWindow<PackageMsgAutoGenEditorWindow>(false, "Message Auto Generation", true);
            window.minSize = new Vector2(750, 100);
            window.maxSize = new Vector2(750, 100);
            window.Show();
        }

        protected override List<string> Generate(string inPath, string outPath, string rosPackageName = "")
        {
            return MessageAutoGen.GenerateSingleMessage(inPath, outPath, rosPackageName);
        }
    }
}