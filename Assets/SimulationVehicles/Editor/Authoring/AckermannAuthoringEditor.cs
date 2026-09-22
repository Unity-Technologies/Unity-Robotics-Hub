using UnityEngine;
using UnityEditor;

namespace Unity.Simulation.VehicleControllers
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(AckermannAuthoring))]
    public class AckermannAuthoringEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();
            
            if (EditorGUI.EndChangeCheck())
            {
                foreach(var target in targets)
                {
                    var authoringComponent = target as AckermannAuthoring;
                    Undo.RecordObject(authoringComponent, $"Setup {authoringComponent.name} Vehicle");
                    authoringComponent.SetupArticulations();
                }
            }
        }
    }
}
