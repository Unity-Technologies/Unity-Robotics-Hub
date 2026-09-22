using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(MastKeyboardAdapter))]
    public class MastKeyboardAdapterEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            EditorGUILayout.HelpBox(
                "This robot is controlled using Unity input axes:\nVertical extension\t\t- Up, Down\nLongitudinal Extension\t- Left, Right\nLateral Extension\t\t- C, V\nTilt\t\t\t- F, R",
                MessageType.Info);
        }
    }
}
