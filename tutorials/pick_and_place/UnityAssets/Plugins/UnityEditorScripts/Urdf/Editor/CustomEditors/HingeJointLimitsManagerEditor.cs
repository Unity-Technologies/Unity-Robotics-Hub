 

using UnityEditor;

namespace RosSharp.Urdf.Editor
{
    
    [CustomEditor(typeof(HingeJointLimitsManager))]
    public class HingeJointLimitsManagerEditor : UnityEditor.Editor
    {
        private const float toleranceThreshold = 10;

        private HingeJointLimitsManager hingeJointLimitsManager;

        public override void OnInspectorGUI()
        {
            base.OnInspectorGUI();

            hingeJointLimitsManager = (HingeJointLimitsManager)target;
            //if (EditorGUILayout.Foldout(true, "Angles"))

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Actual Angle:", hingeJointLimitsManager.AngleActual.ToString());
            EditorGUILayout.LabelField("Actual Rotation No.:", hingeJointLimitsManager.RotationNumberActual.ToString());
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Min Angle:", hingeJointLimitsManager.AngleLimitMin.ToString());
            EditorGUILayout.LabelField("Min. No. of Rotations:", hingeJointLimitsManager.RotationNumberMin.ToString());
            EditorGUILayout.EndHorizontal();

            EditorGUILayout.BeginHorizontal();
            EditorGUILayout.LabelField("Max Angle:", hingeJointLimitsManager.AngleLimitMax.ToString());
            EditorGUILayout.LabelField("Max. No. of Rotations:", hingeJointLimitsManager.RotationNumberMax.ToString());
            EditorGUILayout.EndHorizontal();

            if (180 - hingeJointLimitsManager.AngleLimitMin < toleranceThreshold)
                EditorGUILayout.HelpBox("Min. Angle is close to +180° where this fix will not work properly. Please increase tolerance.", MessageType.Warning);

            if (180 - hingeJointLimitsManager.AngleLimitMax < toleranceThreshold)
                EditorGUILayout.HelpBox("Max. Angle is close to -180° where this fix will not work properly. Please increase tolerance.", MessageType.Warning);

        }
    }
}