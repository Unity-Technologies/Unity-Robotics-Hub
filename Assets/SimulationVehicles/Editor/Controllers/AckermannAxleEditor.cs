using System.Collections;
using System.Collections.Generic;
using UnityEditor;
using UnityEngine;

namespace Unity.Simulation.VehicleControllers
{
    [CanEditMultipleObjects]
    [CustomEditor(typeof(AckermannAxle))]
    public class AckermannAxleEditor : Editor
    {
        public override void OnInspectorGUI()
        {
            DrawDefaultInspector();

            if(GUILayout.Button("Find all components"))
            {
                foreach(var target in targets)
                {
                    var axle = target as AckermannAxle;

                    if (axle == null)
                        return;

                    var controller = axle.Controller;

                    if (!controller)
                        controller = axle.GetComponentInParent<AckermannController>();

                    if (!controller)
                        return;

                    FindAndAssignAxleComponentsWithUndo(controller, axle);
                }
            }
        }

        public static void FindAndAssignAxleComponentsWithUndo(AckermannController controller, AckermannAxle axle)
        {
            Undo.RecordObject(axle, "Find and Assign axle components");

            AckermannAuthoring.FindAndAssignAxleComponents(controller, axle);

            PrefabUtility.RecordPrefabInstancePropertyModifications(axle);
        }
    }
}
