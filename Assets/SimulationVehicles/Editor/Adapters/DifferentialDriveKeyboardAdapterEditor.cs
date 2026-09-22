using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(DifferentialDriveKeyboardAdapter))]
    public class DifferentialDriveKeyboardAdapterEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();
            EditorGUILayout.HelpBox(
                "This robot is controlled using Unity input axes:\n Vertical (Forward/Backward) - W, S \n Horizontal (Twist) - A, D",
                MessageType.Info);
        }
    }
}
