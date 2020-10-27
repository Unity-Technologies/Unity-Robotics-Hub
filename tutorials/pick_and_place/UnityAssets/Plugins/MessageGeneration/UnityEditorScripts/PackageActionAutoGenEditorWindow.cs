using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

 namespace RosMessageGeneration
{
    public class PackageActionAutoGenEditorWindow : PackageAutoGenEditorWindow
    {
        protected override string GenerationType
        {
            get { return "action"; }
        }

        protected override string FileExtension
        {
            get { return "action"; }
        }

        // TODO: Implement and test ROS Action serialization and deserialization
       // [MenuItem("RosMessageGeneration/Auto Generate Actions/Package Actions...", false, 21)]
        private static void OpenWindow()
        {
            PackageActionAutoGenEditorWindow window = GetWindow<PackageActionAutoGenEditorWindow>(false, "Action Auto Generation", true);
            window.minSize = new Vector2(750, 100);
            window.maxSize = new Vector2(750, 100);
            window.Show();
        }

        protected override List<string> Generate(string inPath, string outPath, string rosPackageName = "")
        {
            return ActionAutoGen.GenerateSingleAction(inPath, outPath, rosPackageName);
        }
    }
}

