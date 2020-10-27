using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace RosMessageGeneration
{
    public class DirectoryActionAutoGenEditorWindow : DirectoryAutoGenEditorWindow
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
        //[MenuItem("RosMessageGeneration/Auto Generate Actions/All Actions in directory...", false, 22)]
        private static void OpenWindow()
        {
            DirectoryActionAutoGenEditorWindow window = GetWindow<DirectoryActionAutoGenEditorWindow>(false, "Action Auto Generation", true);
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
