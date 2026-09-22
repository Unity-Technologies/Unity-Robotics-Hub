using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(AckermannKeyboardAdapter))]
    public class AckermannKeyboardAdapterEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            EditorGUILayout.HelpBox(
                "This robot is controlled using Unity input axes:\nVertical (Forward/Backward) - W, S\nHorizontal (Steering)\t- A, D\nBrake\t\t\t- Space",
                MessageType.Info);
        }
    }
}
