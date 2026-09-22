using UnityEngine;
using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(ThreeWheelAuthoring))]
    public class ThreeWheelAuthoringEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            EditorGUI.BeginChangeCheck();

            DrawDefaultInspector();

            if (EditorGUI.EndChangeCheck() && target is ThreeWheelAuthoring threeWheelAuthoring)
                threeWheelAuthoring.SetupArticulations();
        }
    }
}
