using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

 namespace RosMessageGeneration
{
    public class SingleActionAutoGenEditorWindow : SingleAutoGenEditorWindow
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
        //[MenuItem("RosMessageGeneration/Auto Generate Actions/Single Action...", false, 20)]
        private static void OpenWindow()
        {
            SingleActionAutoGenEditorWindow window = GetWindow<SingleActionAutoGenEditorWindow>(false, "Action Auto Generation", true);
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