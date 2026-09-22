using UnityEngine;
using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(MastAuthoring))]
    public class MastAuthoringEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            EditorGUI.BeginChangeCheck();

            DrawDefaultInspector();

            if (EditorGUI.EndChangeCheck() && target is MastAuthoring mastAuthoring)
                mastAuthoring.SetupArticulations();
        }
    }
}
