using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(AckermannDebugAdapter))]
    public class AckermannDebugAdapterEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            var adapter = target as AckermannDebugAdapter;

            EditorGUILayout.HelpBox(
                "This adapter will control all child controllers",
                MessageType.Info);
            
        }
    }
}
