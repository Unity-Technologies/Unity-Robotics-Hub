using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

 namespace RosMessageGeneration
{
    public class DirectorySrvAutoGenEditorWindow : DirectoryAutoGenEditorWindow
    {

        protected override string GenerationType
        {
            get { return "service"; }
        }

        protected override string FileExtension
        {
            get { return "srv"; }
        }


        [MenuItem("RosMessageGeneration/Auto Generate Services/All Services in directory...", false, 12)]
        private static void OpenWindow()
        {
            DirectorySrvAutoGenEditorWindow window = GetWindow<DirectorySrvAutoGenEditorWindow>(false, "Service Auto Generation", true);
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
