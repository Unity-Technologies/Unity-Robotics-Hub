using System;
using UnityEngine;
using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CustomEditor(typeof(DifferentialAuthoring))]
    public class DifferentialAuthoringEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            EditorGUI.BeginChangeCheck();
            DrawDefaultInspector();

            if (EditorGUI.EndChangeCheck() && target is DifferentialAuthoring differentialAuthoring)
                differentialAuthoring.SetupArticulations();
        }
    }
}
